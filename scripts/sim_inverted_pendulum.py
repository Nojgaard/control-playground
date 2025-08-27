from sim.application import Application
from sim.inverted_pendulum import InvertedPendulum
from sim.robot_state import RobotState
import numpy as np

"""
Simulates a cart–pole inverted pendulum system.

The physical model is specified in `mjcf/inverted_pendulum/robot.xml` and
consists of:
 - A cart that can slide horizontally along a track.
 - A pendulum pole hinged to the top of the cart.

It is composed of 1 actuator:
 - A linear motor actuator that applies a horizontal force to the cart via its
   slide joint.

The robot is controlled by a policy function that takes the current state and
returns the desired action. An action in this case is a NumPy array of shape
(1,) representing the commanded motor input for the cart:

    action[0] = horizontal force applied to the cart (in Newtons if the MJCF
                specifies `gear="1"` on the motor)

Positive values push the cart in the +x direction; negative values push it in
the -x direction. Moving the cart causes the attached pole to accelerate and
can be used to balance it upright or swing it up from a hanging position.
"""


def policy(dt: float, state: RobotState) -> np.ndarray:
    return np.array([0.0])


def main():
    pendulum = InvertedPendulum(control_timestep=0.02)
    app = Application(pendulum, policy)
    app.launch()


if __name__ == "__main__":
    main()
