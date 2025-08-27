from sim.application import Application
from sim.roomba import Roomba
from sim.robot_state import RobotState
import numpy as np

"""
Simulates a three-wheeled Roomba-style robot.

The robot is specified in mjcf/roomba/robot.xml. It is composed of 2 actuators:
Two velocity actuators controlling the left and right drive wheels for
differential drive control.

The robot is controlled by a policy function that takes the current state and
returns the desired action. An action in this case is a numpy array of shape
(2,) representing the desired wheel velocities:
 - velocity of left wheel (in radians per second)
 - velocity of right wheel (in radians per second)

For wheel actuators, a positive velocity command will drive the wheel forwards,
while a negative command will drive it backwards.
Differential drive control allows the robot to:
 - Move forward: both wheels same positive velocity
 - Move backward: both wheels same negative velocity
 - Turn left: right wheel faster than left wheel
 - Turn right: left wheel faster than right wheel
 - Rotate in place: wheels equal and opposite velocities
"""


def policy(dt: float, state: RobotState) -> np.ndarray:
    # Implement your policy logic using the state information
    # Example: Simple forward motion
    # return np.array([1.0, 1.0])  # Both wheels forward at 1 rad/s

    # Example: Circular motion
    # return np.array([2.0, 1.0])  # Left wheel faster, causes right turn

    return np.array([2.0, 1.0])


def main():
    roomba = Roomba(control_timestep=0.05)
    app = Application(roomba, policy)
    app.launch()


if __name__ == "__main__":
    main()
