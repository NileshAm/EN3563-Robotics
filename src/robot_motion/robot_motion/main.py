from numpy import array, linalg, average, zeros
from robot_kinematics.utils import splitH, getTraslation, extendTranslation, IdetityMat
from robot_kinematics.Homogenous import (
    H06,
    IK,
    generateH,
    pose_error,
    joint_to_cartesian_motion,
)
from robot_kinematics import s_curve
import pandas as pd
from robot_motion.plotter import plot_velocity_and_acceleration
import matplotlib.pyplot  as plt

# actualAngles = array([20, 30, 50, 80, 70, 90], dtype=float)
# target = H06(*actualAngles)
# initial = actualAngles - 15

# q_solution, converged = IK(
#     target,
#     initial,
#     max_iterations=30,
#     learning_rate=1.0,
#     max_step_degrees=5.0,
#     seed_weight=1.5,
#     position_tolerance=0.1
# )

# final_error = pose_error(q_solution, target)

# print("Target joint angles (deg):", actualAngles)
# print("Solved joint angles (deg):", q_solution)
# print("Converged:", converged)
# print("Position error (mm):", linalg.norm(final_error[:3]))
# print("Rotation error (rad):", linalg.norm(final_error[3:]))
# print("Avg Angle error (deg):", average(abs(q_solution-actualAngles)))
actualAngles = zeros((6,))
# start = generateH([[0, 0,1],[0,-1,0],[1, 0,0]], [356.75,50,603.4])
# ik = IK(start, actualAngles)[0]
# curve = s_curve(actualAngles, ik, 30, 30, True, move_linear=False)
# position = []
# velocity = []
# acc = []
# for state in curve:
#     joint_position = state["position"]
#     xyz_velocity, xyz_acceleration = joint_to_cartesian_motion(
#         joint_position,
#         state["velocity"],
#         state["acceleration"],
#     )

#     position.append(H06(*joint_position)[:3, 3])
#     velocity.append(xyz_velocity)
#     acc.append(xyz_acceleration)

# # print(getTraslation(H06(*curve[5]["position"]), [0,0,0,1]))
# # print(position)
# # print(velocity)
# # print(acc)

# try:
#     plot_velocity_and_acceleration(position, velocity, acc)
#     plt.show()
# except KeyboardInterrupt:
#     pass

print(getTraslation(H06(*zeros(6,)), [0,0,0,1]))
