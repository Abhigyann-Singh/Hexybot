import pybullet as p
import pybullet_data
import time
import os

# === START: Setup ===
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(0.5, 90, 0, [0, 0, 0.1])

# load plane & robot
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF(
    "urdf/Hexybot.urdf",
    basePosition=[0.015, -0.12, 0.5],
    baseOrientation=p.getQuaternionFromEuler([0,0,0]),
    globalScaling=1,
    useFixedBase=False
)

# map link names to joint indices
link_name_to_index = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    link_name_to_index[info[12].decode('utf-8')] = i

# apply joint limits to all revolute joints
typical_limit = 3.14
for name, ji in link_name_to_index.items():
    p.changeDynamics(robot_id, ji,
                     jointLowerLimit=-typical_limit,
                     jointUpperLimit= typical_limit)

p.changeVisualShape(robot_id, -1, rgbaColor=[0.5, 0.5, 1, 1])  # Baby blue color for the base link
light_blue = [0.5, 0.5, 1, 1]
darker_blue = [0.2, 0.2, 0.5, 1]
darkest_blue = [0.1, 0.1, 0.2, 1]
for leg in range(1, 7):
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_1"], rgbaColor=light_blue)
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_2"], rgbaColor=darker_blue)
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_3"], rgbaColor=darkest_blue)
# helper to get 3 joints of leg N
def leg_joints(n):
    return (
        link_name_to_index[f"{n}_1"],  # coxa
        link_name_to_index[f"{n}_2"],  # femur
        link_name_to_index[f"{n}_3"]   # tibia
    )

# set all joints to position control, zero force limits
for ji in link_name_to_index.values():
    p.setJointMotorControl2(robot_id, ji, p.POSITION_CONTROL, force=20)
# === END: Setup ===

# === PARAMETERS ===
STANCE = [0.0, 0.8, -1.4]
LIFT   = 2
SWING  = 1.5
STEP_DURATION = 0.1
TRIPOD1 = [ 2, 5]
TRIPOD2 = [ 3, 6]

# helper: set one leg to a target pose
def set_leg_pose(leg, pose, maxVel=1.0):
    j_coxa, j_femur, j_tibia = leg_joints(leg)
    p.setJointMotorControl2(robot_id, j_coxa,   p.POSITION_CONTROL, targetPosition=pose[0], maxVelocity=maxVel)
    p.setJointMotorControl2(robot_id, j_femur,  p.POSITION_CONTROL, targetPosition=pose[1], maxVelocity=maxVel)
    p.setJointMotorControl2(robot_id, j_tibia,  p.POSITION_CONTROL, targetPosition=pose[2], maxVelocity=maxVel)

# step simulation for a given duration
def step_wait(duration):
    t_end = time.time() + duration
    while time.time() < t_end:
        p.stepSimulation()
        time.sleep(1/240.)

# === Walking sequence ===
def walk_step(tripod, forward=True):
    swing_dir =  SWING if forward else -SWING
    for leg in tripod:
        set_leg_pose(leg, [STANCE[0] + swing_dir, LIFT, STANCE[2]])
    step_wait(STEP_DURATION)
    for leg in tripod:
        set_leg_pose(leg, [STANCE[0] + swing_dir, STANCE[1], STANCE[2]])
    step_wait(STEP_DURATION)
    for leg in tripod:
        set_leg_pose(leg, [STANCE[0] - swing_dir, STANCE[1], STANCE[2]])
    step_wait(STEP_DURATION)
    for leg in tripod:
        set_leg_pose(leg, STANCE)
    step_wait(STEP_DURATION)

def walk(steps=4):
    for i in range(steps):
        walk_step(TRIPOD1, forward=True)
        walk_step(TRIPOD2, forward=True)

# === Rotation sequence ===
def rotate_step(tripod, clockwise=True):
    swing_dir =  SWING if clockwise else -SWING
    for leg in tripod:
        set_leg_pose(leg, [STANCE[0] + swing_dir, STANCE[1], STANCE[2]])
    step_wait(STEP_DURATION)
    for leg in tripod:
        set_leg_pose(leg, STANCE)
    step_wait(STEP_DURATION)

def rotate(rotations=6, clockwise=True):
    for i in range(rotations):
        rotate_step(TRIPOD1, clockwise)
        rotate_step(TRIPOD2, clockwise)

# === MAIN ===
if __name__ == "__main__":
    for leg in range(1,7):
        set_leg_pose(leg, STANCE, maxVel=2.0)
    step_wait(1.0)

    print("Walking forward...")
    walk(steps=6)

    print("In-place rotating clockwise...")
    rotate(rotations=8, clockwise=True)

    print("Done.")
    while True:
        p.stepSimulation()
        time.sleep(1/240.)
