import pybullet as p
import pybullet_data
import time
import os

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


p.setGravity(0, 0, -10)

camera_distance = 0.5
camera_yaw = 90
camera_pitch = 0
camera_target = [0, 0, 0.1]
p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target)


urdf_path = "urdf/Hexybot.urdf"
scale = 1
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF(
    urdf_path,
    basePosition=[0.015, -0.12, 0.5],
    baseOrientation=p.getQuaternionFromEuler([0,0,0]),
    globalScaling=scale,
    useFixedBase=True,
)

link_name_to_index = {p.getBodyInfo(robot_id)[0].decode("utf-8"): -1}
# Set the base link index to baby blue color
p.changeVisualShape(robot_id, link_name_to_index["base_link"], rgbaColor=[0.5, 0.5, 1, 1])  # Baby blue color for the base link
light_blue = [0.5, 0.5, 1, 1]
darker_blue = [0.2, 0.2, 0.5, 1]
darkest_blue = [0.1, 0.1, 0.2, 1]

# p.changeVisualShape(robot_id, link_name_to_index["1_1"], rgbaColor=light_blue)  # Set first leg color to red
# p.changeVisualShape(robot_id, link_name_to_index["2_2"], rgbaColor=darker_blue)  # Set second leg color to green
# p.changeVisualShape(robot_id, link_name_to_index["3_1"], rgbaColor=darkest_blue)  # Set third leg color to blue


joint_num = p.getNumJoints(robot_id)
print(f"Number of joints: {joint_num}")

for i in range(p.getNumJoints(robot_id)):
    joint_info = p.getJointInfo(robot_id, i)
    link_name = joint_info[12].decode("utf-8")
    link_name_to_index[link_name] = i
    print(f"Joint {i}: {link_name}, Type: {joint_info[2]}")

# Get joint indices
revolute1_1 = link_name_to_index["1_1"]
revolute1_2 = link_name_to_index["1_2"]
revolute1_3 = link_name_to_index["1_3"]
revolute2_1 = link_name_to_index["2_1"]
revolute2_2 = link_name_to_index["2_2"]
revolute2_3 = link_name_to_index["2_3"]
revolute3_1 = link_name_to_index["3_1"]
revolute3_2 = link_name_to_index["3_2"]
revolute3_3 = link_name_to_index["3_3"]
revolute4_1 = link_name_to_index["4_1"]
revolute4_2 = link_name_to_index["4_2"]
revolute4_3 = link_name_to_index["4_3"]
revolute5_1 = link_name_to_index["5_1"]
revolute5_2 = link_name_to_index["5_2"]
revolute5_3 = link_name_to_index["5_3"]
revolute6_1 = link_name_to_index["6_1"]
revolute6_2 = link_name_to_index["6_2"]
revolute6_3 = link_name_to_index["6_3"]

p.changeDynamics(robot_id, revolute1_1, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute1_2, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute1_3, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute2_1, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute2_2, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute2_3, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute3_1, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute3_2, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute3_3, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute4_1, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute4_2, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute4_3, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute5_1, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute5_2, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute5_3, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute6_1, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute6_2, jointLowerLimit= -3.14, jointUpperLimit=3.14)
p.changeDynamics(robot_id, revolute6_3, jointLowerLimit= -3.14, jointUpperLimit=3.14)

# Create sliders for velocities for each joint
slider_revolute1_1 = p.addUserDebugParameter("Velocity_1_1", -5, 5, 0)
slider_revolute1_2 = p.addUserDebugParameter("Velocity_1_2", -5, 5, 0)
slider_revolute1_3 = p.addUserDebugParameter("Velocity_1_3", -5, 5, 0)

slider_revolute2_1 = p.addUserDebugParameter("Velocity_2_1", -5, 5, 0)
slider_revolute2_2 = p.addUserDebugParameter("Velocity_2_2", -5, 5, 0)
slider_revolute2_3 = p.addUserDebugParameter("Velocity_2_3", -5, 5, 0)

slider_revolute3_1 = p.addUserDebugParameter("Velocity_3_1", -5, 5, 0)
slider_revolute3_2 = p.addUserDebugParameter("Velocity_3_2", -5, 5, 0)
slider_revolute3_3 = p.addUserDebugParameter("Velocity_3_3", -5, 5, 0)

slider_revolute4_1 = p.addUserDebugParameter("Velocity_4_1", -5, 5, 0)
slider_revolute4_2 = p.addUserDebugParameter("Velocity_4_2", -5, 5, 0)
slider_revolute4_3 = p.addUserDebugParameter("Velocity_4_3", -5, 5, 0)

slider_revolute5_1 = p.addUserDebugParameter("Velocity_5_1", -5, 5, 0)
slider_revolute5_2 = p.addUserDebugParameter("Velocity_5_2", -5, 5, 0)
slider_revolute5_3 = p.addUserDebugParameter("Velocity_5_3", -5, 5, 0)

slider_revolute6_1 = p.addUserDebugParameter("Velocity_6_1", -5, 5, 0)
slider_revolute6_2 = p.addUserDebugParameter("Velocity_6_2", -5, 5, 0)
slider_revolute6_3 = p.addUserDebugParameter("Velocity_6_3", -5, 5, 0)

initial_positions = [0] * 18
joint_indices = [
    revolute1_1, revolute1_2, revolute1_3,
    revolute2_1, revolute2_2, revolute2_3,
    revolute3_1, revolute3_2, revolute3_3,
    revolute4_1, revolute4_2, revolute4_3,
    revolute5_1, revolute5_2, revolute5_3,
    revolute6_1, revolute6_2, revolute6_3
]
# Set colors for all legs: 1_1, 1_2, 1_3 ... 6_1, 6_2, 6_3
for leg in range(1, 7):
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_1"], rgbaColor=light_blue)
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_2"], rgbaColor=darker_blue)
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_3"], rgbaColor=darkest_blue)

for idx, pos in zip(joint_indices, initial_positions):
    p.setJointMotorControl2(robot_id, idx, p.POSITION_CONTROL, targetPosition=pos, force=500)
p.stepSimulation()
time.sleep(1)

while True:
    p.stepSimulation()
    time.sleep(1/240.)

    vel_1_1 = p.readUserDebugParameter(slider_revolute1_1)
    vel_1_2 = p.readUserDebugParameter(slider_revolute1_2)
    vel_1_3 = p.readUserDebugParameter(slider_revolute1_3)

    vel_2_1 = p.readUserDebugParameter(slider_revolute2_1)
    vel_2_2 = p.readUserDebugParameter(slider_revolute2_2)
    vel_2_3 = p.readUserDebugParameter(slider_revolute2_3)

    vel_3_1 = p.readUserDebugParameter(slider_revolute3_1)
    vel_3_2 = p.readUserDebugParameter(slider_revolute3_2)
    vel_3_3 = p.readUserDebugParameter(slider_revolute3_3)

    vel_4_1 = p.readUserDebugParameter(slider_revolute4_1)
    vel_4_2 = p.readUserDebugParameter(slider_revolute4_2)
    vel_4_3 = p.readUserDebugParameter(slider_revolute4_3)

    vel_5_1 = p.readUserDebugParameter(slider_revolute5_1)
    vel_5_2 = p.readUserDebugParameter(slider_revolute5_2)
    vel_5_3 = p.readUserDebugParameter(slider_revolute5_3)

    vel_6_1 = p.readUserDebugParameter(slider_revolute6_1)
    vel_6_2 = p.readUserDebugParameter(slider_revolute6_2)
    vel_6_3 = p.readUserDebugParameter(slider_revolute6_3)

    p.setJointMotorControl2(robot_id, revolute1_1, p.VELOCITY_CONTROL, targetVelocity=vel_1_1, force=500)
    p.setJointMotorControl2(robot_id, revolute1_2, p.VELOCITY_CONTROL, targetVelocity=vel_1_2, force=500)
    p.setJointMotorControl2(robot_id, revolute1_3, p.VELOCITY_CONTROL, targetVelocity=vel_1_3, force=500)

    p.setJointMotorControl2(robot_id, revolute2_1, p.VELOCITY_CONTROL, targetVelocity=vel_2_1, force=5)
    p.setJointMotorControl2(robot_id, revolute2_2, p.VELOCITY_CONTROL, targetVelocity=vel_2_2, force=5)
    p.setJointMotorControl2(robot_id, revolute2_3, p.VELOCITY_CONTROL, targetVelocity=vel_2_3, force=5)

    p.setJointMotorControl2(robot_id, revolute3_1, p.VELOCITY_CONTROL, targetVelocity=vel_3_1, force=5)
    p.setJointMotorControl2(robot_id, revolute3_2, p.VELOCITY_CONTROL, targetVelocity=vel_3_2, force=5)
    p.setJointMotorControl2(robot_id, revolute3_3, p.VELOCITY_CONTROL, targetVelocity=vel_3_3, force=5)

    p.setJointMotorControl2(robot_id, revolute4_1, p.VELOCITY_CONTROL, targetVelocity=vel_4_1, force=5)
    p.setJointMotorControl2(robot_id, revolute4_2, p.VELOCITY_CONTROL, targetVelocity=vel_4_2, force=5)
    p.setJointMotorControl2(robot_id, revolute4_3, p.VELOCITY_CONTROL, targetVelocity=vel_4_3, force=5)

    p.setJointMotorControl2(robot_id, revolute5_1, p.VELOCITY_CONTROL, targetVelocity=vel_5_1, force=5)
    p.setJointMotorControl2(robot_id, revolute5_2, p.VELOCITY_CONTROL, targetVelocity=vel_5_2, force=5)
    p.setJointMotorControl2(robot_id, revolute5_3, p.VELOCITY_CONTROL, targetVelocity=vel_5_3, force=5)

    p.setJointMotorControl2(robot_id, revolute6_1, p.VELOCITY_CONTROL, targetVelocity=vel_6_1, force=5)
    p.setJointMotorControl2(robot_id, revolute6_2, p.VELOCITY_CONTROL, targetVelocity=vel_6_2, force=5)
    p.setJointMotorControl2(robot_id, revolute6_3, p.VELOCITY_CONTROL, targetVelocity=vel_6_3, force=5)
