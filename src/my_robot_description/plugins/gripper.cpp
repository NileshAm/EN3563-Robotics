// Force-driven parallel jaws; objects are held only by collision friction.
#include <atomic>
#include <algorithm>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/Utility.hh>

namespace workcell
{
class Gripper : public gz::sim::System,
                public gz::sim::ISystemConfigure,
                public gz::sim::ISystemPreUpdate
{
  gz::transport::Node node;
  gz::transport::Node::Publisher statePub, posePub;
  std::atomic<bool> close{false};
  gz::sim::Entity model{};
  std::chrono::steady_clock::duration contactSince{};
  std::chrono::steady_clock::duration lastContact{};
  bool contacting{false};
  bool grasped{false};
  std::chrono::steady_clock::duration lastPublish{};
public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model = entity;
    node.Subscribe("/gripper/close", &Gripper::Command, this);
    statePub = node.Advertise<gz::msgs::Boolean>("/gripper/attached");
    posePub = node.Advertise<gz::msgs::Pose>("/box_b/pose");
  }
  void Command(const gz::msgs::Boolean &msg) { close = msg.data(); }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused) return;
    bool jawsClosed = true;
    for (const auto &name : {"left_finger_joint", "right_finger_joint"})
    {
      const auto finger = gz::sim::Model(model).JointByName(ecm, name);
      if (!finger) { jawsClosed = false; continue; }
      auto pos = ecm.Component<gz::sim::components::JointPosition>(finger);
      auto vel = ecm.Component<gz::sim::components::JointVelocity>(finger);
      if (!pos) ecm.CreateComponent(finger, gz::sim::components::JointPosition());
      if (!vel) ecm.CreateComponent(finger, gz::sim::components::JointVelocity());
      const double q = pos && !pos->Data().empty() ? pos->Data()[0] : 0.0;
      const double v = vel && !vel->Data().empty() ? vel->Data()[0] : 0.0;
      jawsClosed = jawsClosed && q > 0.017;
      // 2 mm overtravel maintains ~4.5 N per finger against a 50 mm cube.
      const double target = close ? 0.022 : 0.0;
      const double force = std::clamp(2250.0 * (target - q) - 10.0 * v, -7.5, 7.5);
      auto cmd = ecm.Component<gz::sim::components::JointForceCmd>(finger);
      if (cmd) cmd->Data() = {force};
      else ecm.CreateComponent(finger, gz::sim::components::JointForceCmd({force}));
    }
    const auto flange = gz::sim::Model(model).LinkByName(ecm, "Link6");
    const auto cube = ecm.EntityByComponents(gz::sim::components::Model(),
                                            gz::sim::components::Name("box_b"));
    if (!flange || !cube) return;
    const auto child = gz::sim::Model(cube).LinkByName(ecm, "link");
    if (!child) return;
    const auto boxPose = gz::sim::worldPose(child, ecm);
    const auto boxCollision = ecm.EntityByComponents(
      gz::sim::components::Collision(), gz::sim::components::ParentEntity(child));
    const auto left = gz::sim::Model(model).LinkByName(ecm, "left_finger");
    const auto right = gz::sim::Model(model).LinkByName(ecm, "right_finger");
    bool leftContact = false, rightContact = false;
    ecm.Each<gz::sim::components::ContactSensorData,
             gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &, const auto *data, const auto *parent)
      {
        for (const auto &contact : data->Data().contact())
        {
          if (boxCollision && (contact.collision1().id() == boxCollision ||
                               contact.collision2().id() == boxCollision))
          {
            if (parent->Data() == left) leftContact = true;
            if (parent->Data() == right) rightContact = true;
          }
        }
        return true;
      });
    const bool bilateral = close && jawsClosed && leftContact && rightContact;
    if (!close || info.simTime < lastContact) grasped = false;
    if (!bilateral || info.simTime < contactSince) contacting = false;
    if (bilateral && !contacting)
    {
      contactSince = info.simTime;
      contacting = true;
    }
    if (bilateral)
    {
      lastContact = info.simTime;
      if (info.simTime - contactSince >= std::chrono::milliseconds(100))
        grasped = true;
    }
    // Contact solvers can briefly alternate which finger reports contact.
    // Opening clears feedback immediately; sustained loss clears it in 150 ms.
    if (!bilateral && info.simTime - lastContact >= std::chrono::milliseconds(150))
      grasped = false;
    if (info.simTime < lastPublish) lastPublish = info.simTime;
    if (info.simTime - lastPublish >= std::chrono::milliseconds(20))
    {
      gz::msgs::Boolean state;
      // Legacy topic name: debounced bilateral contact, never a fixed joint.
      state.set_data(grasped);
      statePub.Publish(state);
      gz::msgs::Pose pose;
      gz::msgs::Set(&pose, boxPose);
      posePub.Publish(pose);
      lastPublish = info.simTime;
    }
  }
};
}
GZ_ADD_PLUGIN(workcell::Gripper, gz::sim::System,
              workcell::Gripper::ISystemConfigure,
              workcell::Gripper::ISystemPreUpdate)
