# Control Theory Playground

A hands-on introduction to control theory through physics-based simulation of biped and quadruped robots. This playground provides an interactive environment for learning and experimenting with robotic control algorithms using MuJoCo physics simulation and DeepMind's `dm_control` library.

## Introduction

This project offers two distinct robotic platforms for exploring control theory concepts:

- **Biped Robot**: A wheeled two-legged robot with 4 actuators (2 hip position controllers, 2 wheel velocity controllers)
- **Quadruped Robot**: A four-legged walking robot with 12 position actuators (3 joints per leg: hip, knee, ankle)

Each robot comes with a complete physics model defined in MJCF (MuJoCo XML Format) and 3D mesh files for realistic visualization. You can implement custom control policies by defining simple Python functions that map robot state to desired actions.

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

## Getting Started

### Examining Robot Models

You can explore each robot model directly using MuJoCo's built-in viewer:

```bash
python -m mujoco.viewer --mjcf mjcf/biped/robot.xml
python -m mujoco.viewer --mjcf mjcf/quadruped/robot.xml
```

This is particularly helpful for understanding how each actuator affects the robot's movement. Use the control tab in the viewer to manually adjust each joint and observe the robot's response in real-time.

**Useful viewer shortcuts:**
- **R** or **Backspace** - Reset simulation
- **T** - Toggle robot transparency
- **F** - Show/hide forces applied to the robot

### Running the Simulations

#### Biped Robot

The biped robot features 4 actuators for movement control:
- **Hip joints** (position control): Adjust robot height and maintain balance
- **Wheel joints** (velocity control): Control forward/backward locomotion

```bash
python scripts/sim_biped.py
```

#### Quadruped Robot

The quadruped features 12 position-controlled joints distributed across its four legs:
- **3 joints per leg**: Hip, knee, and ankle
- Allows for 3 DOF per leg

```bash
python scripts/sim_quadruped.py
```

### Implementing Custom Control Policies

Both simulation scripts include a `policy` function that you can customize:

```python
def policy(dt: float, state: RobotState) -> np.ndarray:
    # Access robot state information
    joint_positions = state.joint_positions
    joint_velocities = state.joint_velocities
    orientation = state.orientation  # Euler angles [roll, pitch, yaw]
    angular_velocity = state.angular_velocity
    linear_acceleration = state.linear_acceleration
    
    # Implement your control logic here
    # Return desired actions (joint positions/velocities)
    return np.array([
        0, # Left leg joint position (radians)
        1, # Right leg joint position (radians)
        2, # Left wheel velocity (radians/seconds)
        3, # Right wheel velocity (radians/second)
    ])  # For biped (4 actuators)
    # return np.zeros(12)  # For quadruped (12 actuators)
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
├── mjcf/                    # Robot model definitions
│   ├── biped/robot.xml      # Biped robot MJCF model
│   ├── quadruped/robot.xml  # Quadruped robot MJCF model
│   └── */stl/               # 3D mesh files
├── sim/                     # Core simulation modules
│   ├── application.py       # Main simulation application
│   ├── biped.py            # Biped robot implementation
│   ├── quadruped.py        # Quadruped robot implementation
│   └── robot_state.py      # State observation interface
└── scripts/                 # Example scripts and notebooks
    ├── sim_biped.py        # Biped simulation runner
    ├── sim_quadruped.py    # Quadruped simulation runner
    └── sim_notes.ipynb     # Interactive analysis notebook
```

### Next Steps

- Experiment with different control strategies (PID controllers, state machines, etc.)
- Try implementing walking gaits for the quadruped
- Use the biped for balance control experiments
- Analyze system stability and response characteristics
- Create your own custom robot models

Happy controlling! 🤖