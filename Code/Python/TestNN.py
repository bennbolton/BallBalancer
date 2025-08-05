from Python.Sim import Plant
from stable_baselines3 import PPO
import numpy as np
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from TrainNN import make_env

test_env = DummyVecEnv([make_env])
test_env = VecNormalize.load("vecnormalize.pkl", test_env)
test_env.training = False  # disable normalisation updates
test_env.norm_reward = False  # disable reward normalisation if you want raw reward output

model = PPO.load("ballbot_ppo", env=test_env)

def rl_control_function(_, q, t):
    """ Wrapper for RL model to act as a control function. """
    q_array = np.array(q, dtype=np.float32).reshape(1, -1)  # Ensure correct input shape
    action, _ = model.predict(q_array, deterministic=True)  # Get action from trained model
    return action.item()  # Convert to scalar

testRun = Plant(rl_control_function)
testRun.run(10, inital_state=np.random.uniform(-0.01, 0.01, size=(4,)))
# testRun.run(3, inital_state=[0, 0.0, 0.03, 0.0])
testRun.show()