// Motorized parallel jaws with proximity-gated ideal grasp assistance.
#include <atomic>
#include <algorithm>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
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
  gz::sim::Entity model{}, joint{gz::sim::kNullEntity};
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
      const double target = close ? 0.020 : 0.0;
      const double force = std::clamp(300.0 * (target - q) - 3.0 * v, -5.0, 5.0);
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
    const auto flangePose = gz::sim::worldPose(flange, ecm);
    const auto tip = flangePose.Pos() + flangePose.Rot().RotateVector({0.12, 0, 0});
    const auto top = boxPose.Pos() + boxPose.Rot().RotateVector({0, 0, 0.025});
    if (close && jawsClosed && !joint && tip.Distance(top) < 0.008 &&
        flangePose.Rot().RotateVector({1, 0, 0}).Dot(
          boxPose.Rot().RotateVector({0, 0, -1})) > 0.995)
    {
      joint = ecm.CreateEntity();
      ecm.CreateComponent(joint, gz::sim::components::DetachableJoint(
        {flange, child, "fixed"}));
    }
    if (!close && joint)
    {
      ecm.RequestRemoveEntity(joint);
      joint = gz::sim::kNullEntity;
    }
    if (info.simTime < lastPublish) lastPublish = info.simTime;
    if (info.simTime - lastPublish >= std::chrono::milliseconds(20))
    {
      gz::msgs::Boolean state;
      state.set_data(joint != gz::sim::kNullEntity);
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
