from sim.application import Application
from sim.biped import Biped
from sim.robot_state import RobotState
import numpy as np

"""
Simulates a wheeled biped.

The robot is specified in mjcf/biped/robot.xml. It is composed of 4 actuators: Two position actuators
controlling the height of the robot and another two velocity actuators controlling velocity of the wheels.

The robot is controlled by a policy function that takes the current state and returns the desired action.
An action in this case is a numpy array of shape (4,) representing the desired joint positions and velocities:
 - angle of left hip (in radians)
 - angle of right hip (in radians)
 - velocity of left wheel (in radians per second)
 - velocity of right wheel (in radians per second)

For hip actuators, a positive position command will drive the joint upwards, while a negative command will drive it downwards.
For wheel actuators, a positive velocity command will drive the wheel forwards, while a negative command will drive it backwards.
"""


def policy(dt: float, state: RobotState) -> np.ndarray:
    # Implement your policy logic using the state information
    return np.zeros(4)


def main():
    biped = Biped(control_timestep=0.02)
    app = Application(biped, policy)
    app.launch()


if __name__ == "__main__":
    main()
