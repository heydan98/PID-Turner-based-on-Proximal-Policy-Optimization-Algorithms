"""my_controller controller."""
from controller import Supervisor
from PPO_agent import PPOAgent, Transition

from gym.spaces import Box, Discrete
import numpy as np
from PID import PID
import os


class PID_tunner(Supervisor, PID):
    def __init__(self, kp, ki, kd, setpoint, episode_limit=1000, steps_per_episode=2000, pretrained_model = None):
        Supervisor.__init__(self)
        PID.__init__(self)
        self.observation_space = Box(low=0,
                                     high=10,
                                     shape=(3, 1),
                                     dtype=np.float64)
        #config timestep for webots simulator
        self.timestep = int(self.getBasicTimeStep())
        #set initial PID values
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        #set setpoint for PID controller
        self.SetPoint = setpoint

        self.episode_limit = episode_limit # maximum episode value
        
        self.pretrained_model = pretrained_model #link to pretrained model

        # 0: increase Kp; 1: increase Ki; 2: increase Kd; 3: decrease all
        self.action_space = Discrete(4) 
        
        self.alpha = 0.1 # attenuation constant of Kp, Ki, Kd

        self.steps_per_episode = steps_per_episode # number of steps per episode

        self.episode_score = 0 # current score
        self.episode_score_list = [] # to compute total score

    def sensors_actuators_config(self):
        raise NotImplementedError

    def get_observations(self):
        raise NotImplementedError

    def get_observations(self):
        return [self.Kp, self.Ki, self.Kd]

    def is_done(self):
        raise NotImplementedError

    def apply_action(self, action):
        NotImplementedError

    def get_default_observation(self):
        # a zero vector
        return [0.0 for _ in range(self.observation_space.shape[0])]

    def get_reward(self, action=None):
        ''' reward of current step
            error = sum()
        '''
        if self.step_count == 0:  
            reward_ = self.SetPoint - abs(self.last_error)
        else:
            reward_ = -abs(self.last_error)

        return reward_

    def reset(self):
        self.simulationReset()
        self.simulationResetPhysics()
        self.clear()
        super(Supervisor, self).step(int(self.getBasicTimeStep()))
        return self.get_default_observation()

    def step(self, action):
        self.apply_action(action)
        if super(Supervisor, self).step(self.timestep) == -1:
            exit()

        return (
            self.get_observations(),
            self.get_reward(action),
            self.is_done(),
            self.get_infomation(),
        )

    def solved(self):
        return False

    def get_infomation(self):
        return None

    def render(self, mode='human'):
        pass

    def main(self):
        if self.pretrained_model != None:
            self.load(self.pretrained_model)
        
        os.makedirs("models" , exist_ok= True)

        self.agent = PPOAgent(number_of_inputs=self.observation_space.shape[0], number_of_actor_outputs=self.action_space.n)
        with open("PID.txt", "a") as file:
            file.write(f"Kp, Ki, Kd, episode, score \n")
       
        self.episode_count = 0
      
        while self.episode_count < self.episode_limit:
            observation = self.reset()  # Reset robot and get starting observation
            self.episode_score = 0
            self.step_count = 0
            for self.step_count in range(self.steps_per_episode):
                # In training mode the agent samples from the probability distribution, naturally implementing exploration
                selected_action, action_prob = self.agent.work(observation, type_="selectAction")
                # Step the supervisor to get the current selected_action's reward, the new observation and whether we reached
                # the done condition
                new_observation, reward, check__done, infomation = self.step([selected_action])

                # Save the current state transition in agent's memory
                trans = Transition(observation, selected_action, action_prob, reward, new_observation)
                self.agent.store_transition(trans)

                if check__done:
                    # Save the episode's score
                    self.episode_score_list.append(self.episode_score)
                    self.agent.train_step(batch_size=self.step_count + 1)
                    break

                self.episode_score += reward  # Accumulate episode reward
                # observation for next step is current step's new_observation
                observation = new_observation
            
            with open("PID.txt", "a") as file:
                file.write(f"{self.Kp}, {self.Ki}, {self.Kd}, {self.episode_count}, {self.episode_score} \n")

            if (self.episode_count % 100 == 0) or (self.episode_count == self.episode_limit):
                save_path = f"models/model_{self.episode_count}"
                self.agent.save(save_path)

            print("Episode #", self.episode_count,
                  "score:", self.episode_score)
            self.episode_count += 1  # Increment episode counter
