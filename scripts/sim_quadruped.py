from sim.application import Application
from sim.quadruped import Quadruped
from sim.robot_state import RobotState
import numpy as np


"""
Simulates a quadruped robot.

The robot is specified in mjcf/quadruped/robot.xml. It is composed of 12 position actuators
controlling the joints of the legs. Each leg is composed of 3 joints: hip, knee, and ankle,
allowing for 3 degrees of freedom per leg.

The robot is controlled by a policy function that takes the current state and returns the desired action.
An action in this case is a numpy array of shape (12,) representing the desired joint positions and velocities:
 - angle of left front hip (in radians)
 - angle of left front knee (in radians)
 - angle of left front ankle (in radians)
 - angle of right front hip (in radians)
 - angle of right front knee (in radians)
 - angle of right front ankle (in radians)
 - angle of left back hip (in radians)
 - angle of left back knee (in radians)
 - angle of left back ankle (in radians)
 - angle of right back hip (in radians)
 - angle of right back knee (in radians)
 - angle of right back ankle (in radians)
"""


def policy(dt: float, state: RobotState) -> np.ndarray:
    # Implement your policy logic using the state information
    return np.zeros(12)


def main():
    quadruped = Quadruped(control_timestep=0.02)
    app = Application(quadruped, policy)
    app.launch()


if __name__ == "__main__":
    main()
