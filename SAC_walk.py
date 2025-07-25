import os
import time
import gym
import numpy as np
import pybullet as p
import pybullet_data
from gym import spaces
from stable_baselines3 import SAC
from stable_baselines3.common.evaluation import evaluate_policy


class HexabotEnv(gym.Env):
    """
    Custom Gym environment for a 6-legged hexybot using PyBullet.
    Action: target joint velocities for 18 revolute joints.
    Observation: joint positions (18), joint velocities (18), base orientation (3), base angular velocities (3).
    Reward: forward velocity minus control cost.
    Termination: if robot falls over or max steps reached.
    """

    metadata = {'render.modes': ['human']}

    def __init__(self, renders=False):
        super(HexabotEnv, self).__init__()
        self._renders = renders
        self._time_step = 1.0 / 240.0
        self.max_steps = 1000
        self.current_step = 0

        # Physics client
        if self._renders:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -10, physicsClientId=self.client)
        p.setTimeStep(self._time_step, physicsClientId=self.client)

        # Load plane and robot
        self.plane = p.loadURDF('plane.urdf', physicsClientId=self.client)
        urdf_path = os.path.join(os.getcwd(), 'urdf', 'Hexybot.urdf')
        self.robot = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0.5],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=False,
            physicsClientId=self.client
        )

        # Get joint indices
        self.joint_indices = []
        for i in range(p.getNumJoints(self.robot, physicsClientId=self.client)):
            info = p.getJointInfo(self.robot, i, physicsClientId=self.client)
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE:
                self.joint_indices.append(i)

        n_joints = len(self.joint_indices)

        # Action space: target velocity for each joint
        action_high = np.ones(n_joints) * 5.0
        self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)

        # Observation space
        obs_high = np.concatenate([
            np.ones(n_joints) * np.pi,       # joint positions
            np.ones(n_joints) * 10.0,         # joint velocities
            np.ones(3),                       # base orientation (Euler)
            np.ones(3)                        # base angular velocities
        ])
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)

    def reset(self):
        p.resetSimulation(physicsClientId=self.client)
        p.setGravity(0, 0, -10, physicsClientId=self.client)
        p.setTimeStep(self._time_step, physicsClientId=self.client)
        p.loadURDF('plane.urdf', physicsClientId=self.client)
        self.robot = p.loadURDF(
            os.path.join(os.getcwd(), 'urdf', 'Hexybot.urdf'),
            basePosition=[0, 0, 0.5],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=False,
            physicsClientId=self.client
        )
        # Reset joint states
        for j in self.joint_indices:
            p.resetJointState(self.robot, j, targetValue=0.0, targetVelocity=0.0, physicsClientId=self.client)

        self.current_step = 0
        return self._get_obs()

    def step(self, action):
        # Apply actions as velocity control
        for idx, joint_idx in enumerate(self.joint_indices):
            p.setJointMotorControl2(
                bodyIndex=self.robot,
                jointIndex=joint_idx,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=action[idx],
                force=5.0,
                physicsClientId=self.client
            )

        # Step simulation
        p.stepSimulation(physicsClientId=self.client)
        if self._renders:
            time.sleep(self._time_step)

        obs = self._get_obs()
        reward = self._compute_reward()
        self.current_step += 1

        done = False
        # Terminate if too many steps or robot falls
        if self.current_step >= self.max_steps:
            done = True
        else:
            # check base height
            pos, _ = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
            if pos[2] < 0.1:
                done = True

        info = {}
        return obs, reward, done, info

    def _get_obs(self):
        # Joint positions and velocities
        joint_states = p.getJointStates(self.robot, self.joint_indices, physicsClientId=self.client)
        positions = np.array([s[0] for s in joint_states], dtype=np.float32)
        velocities = np.array([s[1] for s in joint_states], dtype=np.float32)
        # Base orientation and angular velocity
        _, ori = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        base_euler = np.array(p.getEulerFromQuaternion(ori), dtype=np.float32)
        _, ang_vel = p.getBaseVelocity(self.robot, physicsClientId=self.client)
        ang_vel = np.array(ang_vel, dtype=np.float32)
        return np.concatenate([positions, velocities, base_euler, ang_vel])

    def _compute_reward(self):
        # Reward forward velocity along X
        vel, _ = p.getBaseVelocity(self.robot, physicsClientId=self.client)
        forward_vel = vel[0]
        # small control cost penalty
        joint_states = p.getJointStates(self.robot, self.joint_indices, physicsClientId=self.client)
        controls = np.array([abs(s[1]) for s in joint_states], dtype=np.float32)
        control_cost = 0.001 * np.sum(controls)
        return forward_vel - control_cost

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect(physicsClientId=self.client)


if __name__ == '__main__':
    # Create environment
    env = HexabotEnv(renders=False)

    # Create SAC model
    model = SAC(
        policy='MlpPolicy',
        env=env,
        verbose=1,
        batch_size=256,
        learning_rate=3e-4,
        gamma=0.99,
        tau=0.005
    )

    # Train the model
    TIMESTEPS = 1_000_000
    model.learn(total_timesteps=TIMESTEPS)

    # Save the trained model
    save_path = os.path.join(os.getcwd(), 'sac_hexabot')
    model.save(save_path)
    print(f"Model saved at {save_path}")

    # Evaluate the trained policy
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward:.2f} +/- {std_reward:.2f}")

    env.close()
