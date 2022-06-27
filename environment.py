# Class with the definition of the environment of the gripper on CoppeliaSim
# Author: Piero Casusol
# Date: 08-06-2022

# Importing the necessary packages
import numpy as np
import zmqRemoteApi as zmq
import gym
import time

# Defining the reference force for the gripper as a constant
REFERENCE_FORCE = 4
# Defining the maximum velocity of the dc motor in degrees/s at 10 RPM.
MAX_VELOCITY = 10 * (360 / 60)

# Defining the class. It has to be child of Env class from OpenAI gym library.
class GripperEnv(gym.Env):

    #Definint the constructor. It has to call the constructor of the parent class first.
    def __init__(self):
        # Calling the constructor of the parent class.
        super(GripperEnv, self).__init__()
        self.client = zmq.RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        # The gripper only receives three observations: the force for every finger of the gripper and for every axis XYZ. Every force has a range of 0 to 100.
        self.observation_space = gym.spaces.Box(low=0, high=100, shape=(1,))


        # The gripper contains one action: The percentage of the maximum velocity of the DC motor. Due to the motor can move clockwise or counterclockwise, the action has a range from -1 to 1.
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(1,))

        # The enviroment dones when 1000 steps are done.
        self.done = False
        # A counter for the number of steps.
        self.step_counter = 0

    # Defining the reset function. It has to be called when resetting the environment.
    def reset(self):
        self.close()
        # A wait time is added to avoid the simulation to start before the previous simulation is stopped.
        time.sleep(1)

        self.client = zmq.RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.client.setStepping(True)
        # A wait time is added to avoid the simulation to start before the client is set to stepping.
        time.sleep(1)

        # Then, the simulation is started.
        self.sim.startSimulation()
        
        # The first observation is the force for every finger of the gripper.
        observation = self.get_state()

        # The step counter is reseted.
        self.step_counter = 0

        # The done flag is reseted.
        self.done = False

        # Return the first observation.
        return observation


    # Defining the step function. It has to be called when taking a step in the environment.
    def step(self, action):
        # Get the action from the action space.
        action = action[0]
        # Create a dummy info dictionary.
        info = {}
        # Validate the action. The value must be between -1 and 1. If not, raise an error.
        if action < -1 or action > 1:
            raise ValueError('The action must be between -1 and 1.')
        
        # Convert the proportion of the maximum velocity to the velocity of the motor.
        velocity = action * MAX_VELOCITY / 100
        # Set the velocity of the motor with setJointTargetVelocity.
        self.sim.setJointTargetVelocity(self.sim.getObjectHandle('DC_Motor'), velocity)
        # Step the simulation.
        self.client.step()
        # Get the force for every finger of the gripper.
        force = self.get_state()
        # Calculate the reward.
        reward = self.get_reward(force)
        # Increment the step counter.
        self.step_counter += 1
        # Check if the environment is done.
        self.done = self.step_counter == 1000
        # Return the observation, the reward and the info dictionary.
        return force, reward, self.done, info

    # Defining the get_state function. It has to be called when getting the state of the environment.
    def get_state(self):
        # Get the force for every finger of the gripper using ReadForceSensor.
        force1 = self.sim.readForceSensor(self.sim.getObjectHandle('Force_Finger_1'))
        force2 = self.sim.readForceSensor(self.sim.getObjectHandle('Force_Finger_2'))
        force3 = self.sim.readForceSensor(self.sim.getObjectHandle('Force_Finger_3'))

        # Saving the force for every finger of the gripper in a list.
        forces = [force1, force2, force3]
        # For each force, if the value is 0, nothing is done. Else, if the value is a tuple, the second element of that tuple will be a list. Replace the value with the first element of that list.
        for i in range(len(forces)):
            if forces[i] != 0:
                if type(forces[i]) == tuple:
                    forces[i] = forces[i][1][2]

        # Return the forces.
        forces = sum([force*force for force in forces]) ** 0.5
        return forces
    
    # Defining the get_reward function. It has to be called when getting the reward of the environment.
    def get_reward(self, force):
        # When the difference between all forces and the reference force is less than 1e-3, the reward is 1.
        if abs(force - REFERENCE_FORCE) < 1e-3:
            return 1
        # Else, the reward is the negative of the square of the difference between all forces and the reference force.
        else:
            return -(force - REFERENCE_FORCE)**2
    
    # Defining the close function. It has to be called when closing the environment.
    def close(self):
        # Stop the simulation.
        self.sim.stopSimulation()