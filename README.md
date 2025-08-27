# Control Theory Playground

A hands-on introduction to control theory through physics-based simulation of diverse robotic systems. This playground provides an interactive environment for learning and experimenting with robotic control algorithms using MuJoCo physics simulation and DeepMind's `dm_control` library.

## Robot Platforms

This project offers four distinct robotic platforms, each designed to explore different aspects of control theory:

### 🎯 **Inverted Pendulum** (Cart-Pole System)
- **Actuators**: 1 force-controlled linear motor
- **Control Type**: Force control (horizontal cart movement)
- **Applications**: Balance control, PID tuning, linear control theory, stability analysis
- **Physics**: Classic underactuated system requiring active stabilization

### 🤖 **Roomba Robot** (Differential Drive)  
- **Actuators**: 2 velocity-controlled drive wheels + 1 passive caster wheel
- **Control Type**: Velocity control (left/right wheel speeds)
- **Applications**: Mobile robotics, differential drive kinematics, path following, obstacle avoidance
- **Physics**: Non-holonomic wheeled robot with 3-wheel configuration

### 🦵 **Biped Robot** (Wheeled Humanoid)
- **Actuators**: 4 total (2 hip position controllers + 2 wheel velocity controllers)
- **Control Type**: Mixed position/velocity control
- **Applications**: Balance control, wheeled locomotion, hybrid control systems
- **Physics**: Balancing robot with articulated legs and drive wheels

### 🐕 **Quadruped Robot** (Four-Legged Walker)
- **Actuators**: 12 position-controlled joints (3 per leg: hip, knee, ankle)
- **Control Type**: Position control
- **Applications**: Walking gaits, coordination, multi-limb control, terrain adaptation
- **Physics**: Complex multi-body dynamics with ground contact forces

Each robot includes complete physics models (MJCF format), 3D meshes for visualization, and sensor feedback. You implement control policies as Python functions that map robot state to desired actions.

### Key Features

- Physics-based simulation using MuJoCo
- Real-time 3D visualization
- State observation including joint positions, velocities, orientation, and accelerations
- Modular design for easy experimentation with different control strategies
- Jupyter notebook support for data analysis and visualization

## Installation

### Prerequisites

- Python 3.8 or higher
- Windows, macOS, or Linux

### Setup Instructions

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Nojgaard/control-playground
   cd control-playground
   ```

2. **Create a virtual environment (recommended):**
   ```bash
   python -m venv control-env
   # On Windows:
   control-env\Scripts\activate
   # On macOS/Linux:
   source control-env/bin/activate
   ```

3. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

## Quick Start

### Explore Robot Models
Use MuJoCo's built-in viewer to examine each robot's physical structure and actuators:

```bash
python -m mujoco.viewer --mjcf mjcf/inverted_pendulum/robot.xml
python -m mujoco.viewer --mjcf mjcf/roomba/robot.xml  
python -m mujoco.viewer --mjcf mjcf/biped/robot.xml
python -m mujoco.viewer --mjcf mjcf/quadruped/robot.xml
```

**Viewer shortcuts:**
- **R** - Reset simulation  
- **T** - Toggle transparency
- **F** - Show/hide forces
- **Control tab** - Manually adjust actuators

### Run Simulations
Each robot has a dedicated simulation script with customizable control policies:

```bash
python scripts/sim_inverted_pendulum.py    # Cart-pole balance control
python scripts/sim_roomba.py               # Differential drive control
python scripts/sim_biped.py                # Wheeled biped control  
python scripts/sim_quadruped.py            # Quadruped walking control
```

## Implementing Custom Control Policies

All simulation scripts include a `policy` function that you can customize to implement your control algorithms:

```python
def policy(dt: float, state: RobotState) -> np.ndarray:
    # Access comprehensive robot state information
    joint_positions = state.joint_positions      # Current joint angles/positions
    joint_velocities = state.joint_velocities    # Joint angular/linear velocities  
    orientation = state.orientation              # Euler angles [roll, pitch, yaw]
    angular_velocity = state.angular_velocity    # Body angular velocity
    linear_acceleration = state.linear_acceleration  # Body linear acceleration
    
    # Implement your control logic here and return desired actions:
    
    # Inverted Pendulum: [horizontal_force] (1 actuator)
    # return np.array([10.0])  # 10N force to the right
    
    # Roomba: [left_wheel_vel, right_wheel_vel] (2 actuators) 
    # return np.array([2.0, 1.5])  # Different wheel speeds for turning
    
    # Biped: [left_hip_pos, right_hip_pos, left_wheel_vel, right_wheel_vel] (4 actuators)
    # return np.array([0.1, -0.1, 2.0, 2.0])  # Hip positions + wheel velocities
    
    # Quadruped: 12 joint positions [FL_hip, FL_knee, FL_ankle, FR_hip, ...] (12 actuators)
    # return np.zeros(12)  # All joints to zero position
```

### Control Examples

**PID Controller for Inverted Pendulum:**
```python
def policy(dt: float, state: RobotState) -> np.ndarray:
    # PID gains  
    kp, kd = 100.0, 10.0
    
    # Get pendulum angle (pole tilt) and angular velocity
    pole_angle = state.joint_positions[1]      # Pole hinge joint
    pole_velocity = state.joint_velocities[1]
    
    # PID control law to balance pole upright
    force = -kp * pole_angle - kd * pole_velocity
    return np.array([force])
```

**Differential Drive for Roomba:**
```python  
def policy(dt: float, state: RobotState) -> np.ndarray:
    # Simple obstacle avoidance based on orientation
    yaw = state.orientation[2]  # Current heading
    target_yaw = 0.0           # Desired heading
    
    # Proportional steering control
    steering_error = target_yaw - yaw
    base_speed = 2.0
    turn_speed = 3.0 * steering_error
    
    left_wheel = base_speed + turn_speed
    right_wheel = base_speed - turn_speed  
    return np.array([left_wheel, right_wheel])
```

### Interactive Analysis with Jupyter

Explore the simulation data interactively using the provided Jupyter notebook:

```bash
jupyter notebook scripts/sim_notes.ipynb
```

The notebook demonstrates:
- How to run simulations programmatically
- Collecting and plotting state data (e.g., pitch angles over time)
- Creating video recordings of simulations
- Data visualization with matplotlib

### Project Structure

```
control-playground/
├── mjcf/                           # Robot model definitions (MuJoCo XML)
│   ├── inverted_pendulum/robot.xml # Cart-pole system model
│   ├── roomba/robot.xml           # Differential drive robot model  
│   ├── biped/robot.xml            # Wheeled biped model + STL meshes
│   └── quadruped/robot.xml        # Four-legged walker + STL meshes
├── sim/                           # Core simulation modules
│   ├── application.py             # Main simulation application
│   ├── inverted_pendulum.py       # Cart-pole implementation
│   ├── roomba.py                  # Roomba implementation
│   ├── biped.py                   # Biped robot implementation
│   ├── quadruped.py               # Quadruped implementation  
│   └── robot_state.py             # State observation interface
└── scripts/                       # Example scripts and analysis tools
    ├── sim_inverted_pendulum.py   # Cart-pole simulation runner
    ├── sim_roomba.py              # Roomba simulation runner
    ├── sim_biped.py               # Biped simulation runner
    ├── sim_quadruped.py           # Quadruped simulation runner
    └── sim_notes.ipynb            # Interactive Jupyter analysis
```

## Learning Pathways

**Beginner (Linear Control):**
1. Start with **Inverted Pendulum** - Classic control problem with clear physics
2. Implement PID controllers and explore stability margins
3. Experiment with LQR (Linear Quadratic Regulator) control

**Intermediate (Mobile Robotics):** 
1. Move to **Roomba Robot** - Learn differential drive kinematics
2. Implement path following and waypoint navigation
3. Add obstacle avoidance behaviors

**Advanced (Multi-Body Dynamics):**
1. **Biped Robot** - Hybrid position/velocity control systems  
2. **Quadruped Robot** - Complex coordination and gait generation
3. Explore machine learning approaches (reinforcement learning)

### Next Steps

- Experiment with different control strategies (PID, LQR, MPC, neural networks)
- Implement walking gaits and dynamic locomotion for legged robots
- Design trajectory optimization and motion planning algorithms
- Explore system identification and adaptive control techniques  
- Create custom robot models for specialized control problems
- Apply reinforcement learning for complex behaviors

**Happy controlling!** 🤖🎯