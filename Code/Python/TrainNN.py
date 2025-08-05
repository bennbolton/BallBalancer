import gym
from gym import spaces
import numpy as np
from scipy.integrate import solve_ivp
from Python.Sim import Plant
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
# from stable_baselines3.common.monitor import Monitor


class BallbotEnv(gym.Env):
    def __init__(self, plant=None):
        super(BallbotEnv, self).__init__()
        self.plant: Plant = plant
        self.plant.controllerFunc = lambda _,q, t: 0,0
        # Define the observation space (4 state variables: theta, psi, theta_dot, psi_dot)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        
        # Define the action space (single continuous value: angular acceleration)
        self.action_space = spaces.Box(low=-2.0, high=2.0, shape=(1,), dtype=np.float32)
        
        # Time step settings
        self.timestep = 0.001  # 50 Hz control frequency
        self.t = 0  # Current time
        
        # Initial state
        self.state = np.array([0, 0.0, 0.03, 0.0])  # [theta, psi, theta_dot, psi_dot]

    def step(self, action):
        """ Applies action, updates state, and returns reward. """
        # self.plant.controllerFunc = lambda _, q, t: np.clip(action.item(), -2, 2)
        self.plant.controllerFunc = lambda _, q, t: action.item() * 5
        # Integrate dynamics over one timestep
        sol = solve_ivp(self.plant.dynamics, [self.t, self.t + self.timestep], self.state, t_eval=[self.t + self.timestep])
        self.state = sol.y[:, -1]  # Update state
        self.t += self.timestep  # Increment time
        
        # Termination condition (optional)
        done = abs(self.state[1]) > np.pi / 2  # End if bot falls too far

        return np.array(self.state, dtype=np.float32), self.compute_reward(action.item()), done, {}
    
    def compute_reward(self, action):
        theta, theta_dot, psi, psi_dot = self.state
        # Bonus for staying near upright
        upright_bonus = 10.0 if abs(psi) < 0.1 else 0.0

        # Quadratic penalty for tilting
        tilt_penalty = -10 * (psi ** 2)

        # Large penalty for falling over
        fall_penalty = -500 if abs(psi) > np.pi / 2 else 0.0

        velocity_penalty = -0.1 * (abs(theta_dot) + abs(psi_dot))

        # Small reward to incentivise survival
        survival_reward = 1.0

        # Small penalty for performing large actions
        action_penalty = -0.1 * (action ** 2)

        return upright_bonus + tilt_penalty + fall_penalty + velocity_penalty + action_penalty + survival_reward

    def reset(self):
        """ Resets the simulation. """
        self.state = np.random.uniform(-0.1, 0.1, size=(4,))  # Small random initial condition
        self.t = 0  # Reset time
        return np.array(self.state, dtype=np.float32)

    def render(self, mode='human'):
        """ Optional visualization (could add matplotlib animation later). """
        self.plant.show()
        pass

# Create the environment
def make_env():
    system = Plant(control='Accel')
    system.grav *= 0.5
    env = BallbotEnv(plant=system)
    env._max_episode_steps = 5000
    # env = Monitor(env)
    return env

if __name__ =="__main__":

    vec_env = DummyVecEnv([make_env])
    vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True, clip_obs=10.)

    vec_env = VecNormalize.load("vecnormalize.pkl", vec_env)

    # Create and train the RL agent
    # policy_kwargs = {"log_std_init": 0.5}
    # model = PPO("MlpPolicy", env, verbose=1, policy_kwargs=policy_kwargs, ent_coef=0.03, learning_rate=0.001)

    policy_kwargs = dict(log_std_init=0.5, net_arch=[dict(pi=[64, 64], vf=[64, 64])])

    # model = PPO("MlpPolicy", vec_env, 
    #             learning_rate=1e-3, 
    #             ent_coef=0.03, 
    #             policy_kwargs=policy_kwargs, 
    #             verbose=1)


    model = PPO.load("ballbot_ppo")
    model.set_env(vec_env)

    model.learn(total_timesteps=100000)  # Adjust training time as needed

    # Save the trained model
    model.save("ballbot_ppo")
    vec_env.save("vecnormalize.pkl")