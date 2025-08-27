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

### Running the Simulations

#### Biped Robot

The biped robot is controlled through 4 actuators:
- Hip joints (position control): Control robot height and balance
- Wheel joints (velocity control): Control forward/backward movement

```bash
python scripts/sim_biped.py
```

#### Quadruped Robot

The quadruped has 12 position-controlled joints (3 per leg):
- Hip, knee, and ankle joints for each leg
- Enables complex walking gaits and movements

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
    return np.zeros(4)  # For biped (4 actuators)
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