# Robot Collision Detection

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Tests](https://img.shields.io/badge/tests-54%20passed-brightgreen.svg)]()

A Python package for real-time collision detection between industrial robots using simplified geometric models (spheres & capsules), with URDF support and STL mesh visualization.

<table>
  <tr>
    <td><img src="images/collision_visualization.png" alt="Dual robot collision simulation" width="1000"/></td>
  </tr>
  <tr>
    <td><i>Dual robot collision simulation showing minimum distance over time (left) and robot configuration at collision point (right)</i></td>
  </tr>
</table>

## Features

**Collision Detection**
- Vectorized distance computation between sphere-sphere, sphere-capsule, and capsule-capsule pairs (7.6x faster than naive loops)
- Early termination with configurable threshold for real-time applications
- Self-collision detection with adjacency-aware link filtering
- Dual-robot and multi-robot collision checking

**Robot Modeling**
- URDF robot model loading with automatic kinematic chain extraction
- STL collision mesh loading and 3D visualization
- Auto-fitted per-link capsule radii from mesh bounding boxes
- DH parameter-based robot definition with custom geometric models
- Forward kinematics for arbitrary N-DOF robots

**Visualization**
- 3D mesh rendering of URDF collision/visual geometry
- Joint markers, coordinate frames, and kinematic skeleton overlay
- Distance-time curve plotting for trajectory analysis

<table>
  <tr>
    <td><img src="images/urdf_franka_mesh.png" alt="Franka Panda URDF mesh visualization" width="1000"/></td>
  </tr>
  <tr>
    <td><i>Franka Panda loaded from URDF: zero configuration (left), working configuration (center), and dual-robot collision check (right)</i></td>
  </tr>
</table>

## Installation

```bash
git clone https://github.com/10-OASIS-01/robot-collision-detection.git
cd robot-collision-detection
pip install -e ".[dev]"
```

## Quick Start

### Dual-Robot Collision Detection (DH Parameters)

```python
import numpy as np
from robot_collision_detection import Robot, min_distance_between_robots

# Define robot with DH parameters [a, alpha, d]
dh_params = [[0, 0, 1100], [600, np.pi/2, 0], [1400, 0, 0]]

robot1 = Robot(dh_params, name="R1")
robot2 = Robot(dh_params, base_transform=T_base, name="R2")

# Update joint configuration
angles = np.radians([0, -45, -20])
transforms = robot1.forward_kinematics(angles)
robot1.update_geometric_model(transforms)

# Check collision
dist, coll_type, elements = min_distance_between_robots(robot1, robot2)
print(f"Distance: {dist:.2f}mm, Type: {coll_type}")
```

### Loading a URDF Robot

```python
from robot_collision_detection import URDFRobot, min_distance_between_robots

# Load from URDF with auto-fitted collision radii
robot = URDFRobot("path/to/robot.urdf", auto_radius=True)
print(f"{robot.name}: {robot.num_joints} joints")

# Forward kinematics
joint_angles = np.zeros(robot.num_joints)
transforms = robot.forward_kinematics(joint_angles)
robot.update_geometric_model(transforms)

# Collision detection
dist, coll_type, elements = min_distance_between_robots(robot1, robot2)

# Early termination (return as soon as collision found)
dist, _, _ = min_distance_between_robots(robot1, robot2, threshold=0.0)
```

### Self-Collision Detection

```python
from robot_collision_detection import URDFRobot, min_distance_within_robot

robot = URDFRobot("franka.urdf", auto_radius=True)
transforms = robot.forward_kinematics(joint_angles)
robot.update_geometric_model(transforms)

# Check non-adjacent links (skip_adjacent=1 ignores directly connected links)
dist, coll_type, elements = min_distance_within_robot(robot, skip_adjacent=1)
if dist < 0:
    print(f"Self-collision between {elements[0]} and {elements[1]}")
```

### Visualization

```python
import matplotlib.pyplot as plt
from robot_collision_detection import URDFRobot
from robot_collision_detection.visualization.plotting import plot_urdf_robot

robot = URDFRobot("franka.urdf")
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Render STL collision meshes with joint markers and coordinate frames
plot_urdf_robot(ax, robot, joint_angles, color='steelblue',
                show_mesh='collision', show_frames=True)
plt.show()
```

## Examples

| Script | Description |
|--------|-------------|
| `examples/base_dual_robot_model.py` | Basic dual-robot setup and visualization |
| `examples/dual_robot_simulation.py` | Dynamic trajectory simulation with collision detection |
| `examples/distance_calculation.py` | Distance computation between geometric primitives |
| `examples/custom_robot_model.py` | Custom geometric models with precision/speed tradeoffs |
| `examples/urdf_robot_example.py` | URDF loading, mesh visualization, and dual-robot check |

```bash
python examples/dual_robot_simulation.py
python examples/urdf_robot_example.py path/to/robot.urdf
```

## Package Structure

```
robot_collision_detection/
├── core/
│   ├── robot.py              # Robot class (DH parameters)
│   ├── kinematics.py         # DH transformation matrices
│   ├── urdf_loader.py        # URDF parser, URDFRobot class
│   └── stl_loader.py         # Binary STL mesh loader
├── distance/
│   ├── primitives.py         # Sphere/capsule/segment distance functions
│   └── collision.py          # Vectorized collision detection & self-collision
├── visualization/
│   └── plotting.py           # 3D plotting, mesh rendering, coordinate frames
└── tests/
    ├── test_primitives.py    # Distance function tests (incl. edge cases)
    ├── test_kinematics.py    # DH transform verification
    ├── test_robot.py         # Forward kinematics & geometric model tests
    ├── test_urdf_loader.py   # URDF parsing & joint chain tests
    └── test_collision.py     # Vectorized collision & self-collision tests
```

## Algorithm

**For a detailed explanation of the mathematical foundations, see the [Algorithm Tutorial](algorithm_tutorial.md).**

The collision detection pipeline:

1. Each robot link is wrapped in spheres and capsules (auto-fitted from URDF mesh or manually defined)
2. Forward kinematics positions all primitives in world coordinates
3. Vectorized numpy broadcasts compute all pairwise distances in one pass:
   - Sphere-sphere: `||c1 - c2|| - r1 - r2`
   - Sphere-capsule: point-to-segment distance minus radii
   - Capsule-capsule: segment-to-segment distance (two-pass clamping) minus radii
4. Optional early termination returns immediately when any distance < threshold
5. Self-collision checks mask adjacent link pairs before searching

<table>
  <tr>
    <td><img src="images/distance_calculations.png" alt="Distance calculations between geometric primitives" width="1000"/></td>
  </tr>
  <tr>
    <td><i>Distance calculations between geometric primitives: sphere-sphere (left), sphere-capsule (middle), and capsule-capsule (right)</i></td>
  </tr>
</table>

## Performance

Benchmarked on Franka Panda (19 primitives per robot, 361 pairs):

| Operation | Time |
|-----------|------|
| Dual-robot collision (vectorized) | **0.16 ms** |
| Dual-robot with early termination | **0.17 ms** |
| Self-collision detection | **0.15 ms** |

7.6x faster than the naive Python-loop implementation.

## API Reference

| Function / Class | Description |
|------------------|-------------|
| `Robot(dh_params, base_transform, name)` | Robot from DH parameters |
| `URDFRobot(urdf_path, capsule_radius, auto_radius)` | Robot from URDF file |
| `min_distance_between_robots(r1, r2, threshold)` | Minimum distance between two robots |
| `min_distance_within_robot(robot, skip_adjacent, threshold)` | Self-collision check |
| `plot_robot(ax, robot, angles, color)` | Plot robot with collision primitives |
| `plot_urdf_robot(ax, robot, angles, show_mesh, show_frames)` | Plot URDF robot with STL meshes |

## Testing

```bash
pip install -e ".[dev]"
pytest tests/ -v
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## References

1. Ericson, C. (2005). *Real-Time Collision Detection*. Morgan Kaufmann Publishers.
2. Gilbert, E., et al. (1988). *A Fast Procedure for Computing the Distance Between Complex Objects in Three-Dimensional Space*. IEEE Journal of Robotics and Automation, 4(2), 193-203.
3. Lin, M.C., et al. (2000). *Fast Proximity Queries with Swept Sphere Volumes*. Proceedings of IEEE International Conference on Robotics and Automation, 3719-3726.
4. Cameron, S. (1997). *Enhancing GJK: Computing minimum and penetration distances between convex polyhedra*. Proceedings of International Conference on Robotics and Automation, 3112-3117.
5. Hunt, K.H. (1978). *Kinematic geometry of mechanisms*. Oxford University Press.
