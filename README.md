# Self-Driving Car Maze Navigation

An interactive Pygame simulation in which a self-driving car navigates from the bottom-left corner to the top-right corner of a randomised obstacle maze.  Four classic navigation algorithms are available to choose from at runtime.

The project is also structured as a Jupyter-notebook exercise: students implement the algorithms in a guided notebook and export their code directly into the simulation for live testing.

---

## Table of Contents

- [Demo](#demo)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Simulation Controls](#simulation-controls)
- [Navigation Algorithms](#navigation-algorithms)
  - [Bug2](#bug2)
  - [Pure Pursuit](#pure-pursuit)
  - [Stanley Controller](#stanley-controller)
  - [Potential Field](#potential-field)
- [Jupyter Notebook Exercise](#jupyter-notebook-exercise)
- [API Reference](#api-reference)
- [Coordinate System](#coordinate-system)

---

## Demo

```
┌─────────────────────────────┬──────────────┐
│                        GOAL │  Self-Driving │
│  ░░░░░   ░░░░░░             │  Car Sim      │
│          ░░░░░░   ░░░░░░░   │               │
│  ░░░░░░            ░░░░░░   │  Algorithm:   │
│            ░░░░░░           │ [Pure Pursuit]│
│  ░░░░░░░            ░░░░░   │               │
│            ░░░░░░░          │  [Reset Maze] │
│  ░░░░░░░            ░░░░░   │               │
│  START                      │  Pos X: 42    │
└─────────────────────────────┴──────────────┘
```

The car (blue rectangle) drives from **START** (bottom-left) to **GOAL** (top-right), leaving a blue trail.  Grey rectangles are randomly placed obstacles.

---

## Project Structure

```
Navigation/
├── simulation.py                  # Pygame application — main entry point
├── algorithms.py                  # Four navigation algorithm implementations
├── navigation_maze.ipynb          # Student notebook (functions left blank)
├── navigation_maze_solution.ipynb # Solution notebook (fully implemented)
└── README.md                      # This file
```

### File roles

| File | Description |
|------|-------------|
| `simulation.py` | 900 × 700 Pygame window with sidebar UI.  Imports algorithm functions from `algorithms.py` and calls the selected one every frame. |
| `algorithms.py` | Contains `bug2_navigate`, `pure_pursuit_navigate`, `stanley_navigate`, `potential_field_navigate`, `_angle_diff`, and `reset_algorithm_state`.  This file is what the student notebooks export to. |
| `navigation_maze.ipynb` | Guided exercise notebook.  Each algorithm has a markdown explanation cell followed by a code cell with a blank skeleton and `TODO` comments. |
| `navigation_maze_solution.ipynb` | Complete implementations of all four algorithms.  Identical export cell to the student notebook. |

---

## Prerequisites

- Python 3.8 or later
- [Pygame](https://www.pygame.org/) 2.x
- NumPy (used in algorithm helpers)
- Jupyter (only needed for the notebook exercise)

Install everything at once:

```bash
pip install pygame numpy jupyter
```

---

## Quick Start

```bash
# Clone the repository
git clone https://github.com/VikramRadhakrishnan/SelfDrivingVehicles-Navigation.git
cd SelfDrivingVehicles-Navigation

# Install dependencies
pip install pygame numpy

# Run the simulation
python simulation.py
```

The simulation window opens immediately.  Use the sidebar dropdown to choose an algorithm and the **Reset Maze** button to randomise the obstacles.

---

## Simulation Controls

| Control | Action |
|---------|--------|
| **Algorithm dropdown** | Switch between Bug2, Pure Pursuit, Stanley, and Potential Field |
| **Reset Maze** button | Randomise obstacles and restart the car from the starting position |
| `Esc` | Quit the simulation |

The sidebar also displays:

- **Pos X / Pos Y** — current car position in pixels
- **Angle** — current heading in degrees
- **Dist** — straight-line distance to the goal in pixels
- **Status** — current algorithm mode or a "Goal reached!" message

---

## Navigation Algorithms

All four algorithms share the same function signature:

```python
def algorithm_navigate(
    car_x: float, car_y: float, car_angle: float,
    goal_x: float, goal_y: float,
    obstacles: list[tuple[int, int, int, int]],
    car_width: float, car_height: float,
    dt: float,
) -> tuple[float, float, float]:   # (new_x, new_y, new_angle)
```

Each function maintains a small `_state` dict as a function attribute so it can track mode, cached waypoints, or other data across frames.  `reset_algorithm_state()` clears all state dicts when the maze is reset.

---

### Bug2

**File:** `algorithms.py` → `bug2_navigate`

Bug2 is a provably complete reactive navigation algorithm that requires no global map.

**How it works:**

1. **Go-to-goal mode** — drive straight along the *M-line* (the line connecting the starting position to the goal).
2. When an obstacle is detected ahead, record the *hit-point* and switch to **boundary-following mode** — trace the obstacle perimeter.
3. Once the M-line is re-encountered at a point *closer to the goal* than the hit-point, switch back to go-to-goal.

**Strengths:** Simple, low memory, guaranteed to reach goal for simple connected obstacles.  
**Weaknesses:** Can take a very long path; may struggle with concave obstacles or narrow corridors.

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPEED` | 80 px/s | Forward speed |
| `TURN_RATE` | 2.5 rad/s | Maximum steering rate |
| `SENSOR_DIST` | 40 px | Look-ahead distance for obstacle detection |
| `OBSTACLE_MARGIN` | 18 px | Extra clearance around obstacle bounding boxes |

---

### Pure Pursuit

**File:** `algorithms.py` → `pure_pursuit_navigate`

Pure Pursuit is a classic path-tracking algorithm originally designed for autonomous ground vehicles.

**How it works:**

1. A greedy waypoint planner builds a collision-free path from the car to the goal (stepping 50 px at a time, trying headings ±0.4 rad, ±0.8 rad, … until a clear step is found).
2. Each frame, a *lookahead point* is found on the path at a fixed distance `LOOKAHEAD` ahead of the car.
3. The car steers toward the lookahead point at a clamped turn rate and moves forward at constant speed.
4. If the new position collides with an obstacle the waypoint cache is cleared and a new path is planned next frame.

**Strengths:** Smooth trajectories; easy to tune via `LOOKAHEAD`.  
**Weaknesses:** Path quality depends entirely on the greedy planner; large `LOOKAHEAD` cuts corners.

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPEED` | 90 px/s | Forward speed |
| `LOOKAHEAD` | 60 px | Look-ahead distance on path |
| `TURN_RATE` | 3.0 rad/s | Maximum steering rate |
| `WAYPOINT_REACH` | 30 px | Radius within which a waypoint is considered reached |

---

### Stanley Controller

**File:** `algorithms.py` → `stanley_navigate`

The Stanley controller was used by Stanford's *Stanley* robot to win the 2005 DARPA Grand Challenge.

**How it works:**

Uses the same greedy waypoint path as Pure Pursuit, but the steering law combines two error terms:

$$\delta = \psi_e + \arctan\!\left(\frac{k \cdot e}{v}\right)$$

- **Heading error** $\psi_e$: difference between the car's heading and the path tangent direction.
- **Cross-track error** $e$: signed perpendicular distance from the car to the nearest path segment (positive = car is left of path).
- $k$: gain; $v$: speed.

At high speed the cross-track correction is small (smooth); at low speed it can be large (aggressive re-centering).

**Strengths:** Naturally balances heading alignment and lateral error; well understood analytically.  
**Weaknesses:** Sensitive to gain tuning; can oscillate at low speed if `K` is too large.

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPEED` | 85 px/s | Forward speed |
| `K` | 2.0 | Cross-track error gain |
| `TURN_RATE` | 3.5 rad/s | Maximum steering rate |
| `WAYPOINT_REACH` | 25 px | Radius within which a waypoint is considered reached |

---

### Potential Field

**File:** `algorithms.py` → `potential_field_navigate`

Potential Field navigation treats the environment as a scalar field and moves the robot along its negative gradient.

**How it works:**

- The goal exerts an **attractive force**: $\mathbf{F}_{att} = k_{att} \cdot (\mathbf{q}_{goal} - \mathbf{q})$
- Each obstacle exerts a **repulsive force** when the robot is within influence radius $d_0$:

$$\mathbf{F}_{rep} = k_{rep} \left(\frac{1}{d} - \frac{1}{d_0}\right) \frac{1}{d^2} \hat{\mathbf{d}} \quad \text{if } d < d_0$$

where $d$ is the distance to the nearest point on the obstacle surface and $\hat{\mathbf{d}}$ is the unit vector away from it.

The total force is normalised to a direction, and the car steers toward that direction.

**Strengths:** No path planning needed; naturally smooth around obstacles.  
**Weaknesses:** Can get stuck in **local minima** (where attractive and repulsive forces cancel).  A small perturbation fallback is included to escape shallow minima.

**Key parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPEED` | 75 px/s | Forward speed |
| `K_ATT` | 1.0 | Attractive force gain |
| `K_REP` | 8000.0 | Repulsive force gain |
| `D0` | 60 px | Obstacle influence radius |
| `TURN_RATE` | 3.0 rad/s | Maximum steering rate |

---

## Jupyter Notebook Exercise

The project includes a self-contained guided exercise for students learning navigation algorithms.

### Workflow

1. Open the student notebook:
   ```bash
   jupyter notebook navigation_maze.ipynb
   ```
2. Read the markdown explanation for each algorithm.
3. Fill in the `TODO` sections in each function skeleton.
4. Run the **Export** cell at the bottom of the notebook:
   ```python
   # This cell inspects each function with `inspect.getsource` and writes
   # algorithms.py in the same directory.
   ```
5. Launch the simulation to test your implementation:
   ```bash
   python simulation.py
   ```
6. Use **Reset Maze** repeatedly to verify robustness across different obstacle layouts.

If you get stuck, consult `navigation_maze_solution.ipynb` for the reference implementation.

### What students implement

Each function skeleton includes:
- Tunable constants with sensible defaults (do not need to be changed)
- Pre-initialised `_state` dict with the right keys
- Inner helper function stubs (`point_in_obstacle`, `point_clear`, `path_clear`, `build_waypoints`, etc.) with docstrings explaining what each should do
- `TODO` comments marking every piece that needs to be written
- A fallback `return car_x, car_y, car_angle` so the notebook exports without syntax errors even before the TODOs are filled in

---

## API Reference

### `algorithms.py`

#### `bug2_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt)`
Bug2 reactive navigation. Returns `(new_x, new_y, new_angle)`.

#### `pure_pursuit_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt)`
Pure Pursuit path tracker. Returns `(new_x, new_y, new_angle)`.

#### `stanley_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt)`
Stanley controller. Returns `(new_x, new_y, new_angle)`.

#### `potential_field_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt)`
Potential field controller. Returns `(new_x, new_y, new_angle)`.

#### `reset_algorithm_state()`
Clears all per-algorithm `_state` dicts. Call this whenever the maze is reset so algorithms start fresh.

#### `_angle_diff(target, current)`
Returns the signed shortest angular difference `(target - current)` normalised to `[-π, π]`.

---

## Coordinate System

Pygame's y-axis increases **downward**.  All angles are measured in **radians** from the positive x-axis in this screen coordinate system:

- `angle = 0` → car faces right
- `angle = π/2` → car faces **down** (increasing y)
- `angle = -π/2` → car faces **up** (decreasing y)

Consequently, to compute the angle toward a target `(tx, ty)` from `(cx, cy)`:

```python
angle = math.atan2(-(ty - cy), tx - cx)
```

The minus sign on the y-component accounts for the flipped axis.

When moving forward:

```python
new_x = car_x + math.cos(angle) * speed * dt
new_y = car_y - math.sin(angle) * speed * dt   # minus: moving "up" decreases y
```
