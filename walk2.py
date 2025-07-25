import pybullet as p
import pybullet_data
import time
import math

# === SETUP ===
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(1.0, 90, -30, [0, 0, 0.2])

# Load plane and robot URDF
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF(
    "urdf/Hexybot.urdf",
    basePosition=[0, 0, 0.2],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=False
)

# Map link names to joint indices
link_name_to_index = {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    name = info[12].decode('utf-8')
    link_name_to_index[name] = i
print("Link names and indices:")
for name, index in link_name_to_index.items():
    print(f"{name}: {index}")
# Apply joint limits to all revolute joints
limit = math.radians(180)
for ji in link_name_to_index.values():
    p.changeDynamics(robot_id, ji, jointLowerLimit=-limit, jointUpperLimit=limit)

# Set the base link index to baby blue color
p.changeVisualShape(robot_id, -1, rgbaColor=[0.5, 0.5, 1, 1])  # Baby blue color for the base link
light_blue = [0.5, 0.5, 1, 1]
darker_blue = [0.2, 0.2, 0.5, 1]
darkest_blue = [0.1, 0.1, 0.2, 1]
for leg in range(1, 7):
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_1"], rgbaColor=light_blue)
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_2"], rgbaColor=darker_blue)
    p.changeVisualShape(robot_id, link_name_to_index[f"{leg}_3"], rgbaColor=darkest_blue)
# Helper: get joints of leg N
def leg_joints(n):
    return (
        link_name_to_index[f"{n}_1"],  # coxa
        link_name_to_index[f"{n}_2"],  # femur
        link_name_to_index[f"{n}_3"]   # tibia
    )

# Set all joints to position control (no force)
for ji in link_name_to_index.values():
    p.setJointMotorControl2(robot_id, ji, p.POSITION_CONTROL, force=3)

STANCE    = [0.0, 0.7, -1.3]  # [coxa, femur, tibia]
LIFT      = 1              
SWING     = 0.9              
STEP_SIZE = 0.04              
DURATION  = 0.1              

LEFT_LEGS  = {1, 6, 5}
RIGHT_LEGS = {2, 4, 3}

def swing_for_leg(leg):
    return SWING if leg in LEFT_LEGS else -SWING

LIFT_PAIRS    = [(2, 5), (6,3)]
SUPPORT_PAIRS = [(2,5), (6,3)]

def set_leg_pose(leg, pose, vel=5):
    j0, j1, j2 = leg_joints(leg)
    p.setJointMotorControl2(robot_id, j0, p.POSITION_CONTROL,
                            targetPosition=pose[0], maxVelocity=vel)
    p.setJointMotorControl2(robot_id, j1, p.POSITION_CONTROL,
                            targetPosition=pose[1], maxVelocity=vel)
    p.setJointMotorControl2(robot_id, j2, p.POSITION_CONTROL,
                            targetPosition=pose[2], maxVelocity=vel)

# Step simulation for duration dt
def wait(dt):
    end = time.time() + dt
    while time.time() < end:
        p.stepSimulation()
        time.sleep(1/240.)

# Ripple gait: lift, swing, plant, return
def ripple_step(phase_index):
    lift    = LIFT_PAIRS[phase_index]
    support = SUPPORT_PAIRS[phase_index]
    # 1) lift legs
    for leg in lift:
        set_leg_pose(leg, [STANCE[0], LIFT, STANCE[2]])
    wait(DURATION)
    # 2) swing coxa in opposite directions
    for leg in lift:
        signed = swing_for_leg(leg)
        set_leg_pose(leg, [STANCE[0] + signed, LIFT, STANCE[2]])
    wait(DURATION)
    # 3) plant legs (femur down, keep coxa swung)
    for leg in lift:
        signed = swing_for_leg(leg)
        set_leg_pose(leg, [STANCE[0] + signed, STANCE[1], STANCE[2]])
    wait(DURATION)
    # 4) return coxa to neutral stance
    for leg in lift:
        set_leg_pose(leg, STANCE)
    wait(DURATION)

# Perform ripple gait for given cycles
def walk(cycles=3):
    for i in range(cycles * len(LIFT_PAIRS)):
        ripple_step(i % len(LIFT_PAIRS))

        # Set friction for all tibia links (1_3, 2_3, ..., 6_3)
TIBIA_LINKS = [link_name_to_index[f"{n}_3"] for n in range(1, 7)]
for link in TIBIA_LINKS:
    p.changeDynamics(robot_id, link, lateralFriction=2)
        # Set friction for the ground as well (optional)
    p.changeDynamics(plane_id, -1, lateralFriction=2)
# === MAIN ===
if __name__ == '__main__':
    # Move all legs to initial stance
    
    for leg in range(1, 7):
        set_leg_pose(leg, STANCE)
    wait(1.0)
    set_leg_pose(1, [STANCE[0] , LIFT, STANCE[2]] )  # Set first leg to stance quickly
    set_leg_pose(4, [STANCE[0] , LIFT, STANCE[2]])  # Set second leg to stance quickly

    print("Starting ripple gait (forward)")
    walk(cycles=15)
    print("Done.")
 
    # Print current base position and orientation
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    print(f"Base position: {pos}, orientation (quaternion): {orn}")
    # Keep GUI alive
    while True:
        p.stepSimulation()
        time.sleep(1/240.)
