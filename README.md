![Project Screenshot](https://github.com/user-attachments/assets/58fc6da1-4b60-4906-a448-17c98d5ac82c)
# Stewart-Gogh Platform Kinematics and Visualization

## Overview
This project implements a **Hexapod** robot simulation using Python. The hexapod is a six-legged robotic platform that can perform **forward kinematics (FK)** and **inverse kinematics (IK)** computations. The program also provides visualization tools to render the hexapod's structure in 3D using Matplotlib.

## Features
- **Hexapod Initialization:** Creates a six-legged robot with configurable body and base dimensions.
- **Inverse Kinematics (IK):** Computes the required leg positions for a given robot pose.
- **Forward Kinematics (FK):** Estimates the robot's pose given a set of leg lengths.
- **3D Visualization:** Displays the hexapod structure in a 3D space using Matplotlib.
- **Transformation Matrices:** Computes and applies transformations for robot motion simulation.

## Installation
### Prerequisites
Ensure you have **Python 3.x** installed along with the required libraries:
```sh
pip install numpy matplotlib
```

## Usage
### Creating an Instance
```python
from hexapod import Hexapod
my_hexa = Hexapod()
```

### Performing Inverse Kinematics (IK)
```python
pose = [10, 0, 100, 5, 5, 0]  # [x, y, z, roll, pitch, yaw]
leg_vectors = my_hexa.ik(pose)
```

### Performing Forward Kinematics (FK)
```python
leg_lengths = [250.173, 247.707, 253.307, 277.633, 278.454, 254.332]
error_threshold = 0.5
initial_guess = [0, 0, 150, 0, 0, 0]
computed_pose = my_hexa.fk(leg_lengths, error_threshold, initial_guess)
```

### Visualizing the Hexapod
```python
my_hexa.show_robot()
```

## Class Methods
| Method | Description |
|--------|-------------|
| `__init__()` | Initializes the hexapod with default or specified parameters. |
| `ik(pose)` | Computes the required leg vectors to achieve a target pose. |
| `fk(leg_lengths, error, guess_pose)` | Estimates the robot’s pose given leg lengths. |
| `show_robot()` | Renders the hexapod in a 3D Matplotlib plot. |

## Future Improvements
- Add real-time simulation with animated movement.
- Implement a physics-based model for realistic motion.
- Improve inverse kinematics with numerical solvers.

## License
This project is licensed under the **MIT License**.


