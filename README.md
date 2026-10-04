## Path Planning Assignment — Solution

### Task Description

#### Part 1

Given up to **two cones** (sometimes only one or none), each with coordinates `(x, y)` and a `color` flag:
- `color == 0`: yellow cone (right side of the track)
- `color == 1`: blue cone (left side of the track)

And the car pose `(x, y, yaw)`, the goal is to return a sequence of path points `(x, y)` in world coordinates representing a drivable route between the boundaries.

#### Part 2

Extend the algorithm to handle **three cones on one side** of the track.

---

### Quick Start

1. Install Python 3.9+.
2. Create a virtual environment and install dependencies:

```bash
python3.9 -m venv .venv
source .venv/bin/activate   # Linux/macOS
pip install -r requirements.txt
pip install PyQt5           # for interactive window display
```

3. Run a scenario:

```bash
DISPLAY=:0 python -m src.run --scenario 2
```

Available scenarios: `1` to `25`.  
Scenarios `1`–`20` are original (up to 2 cones per side).  
Scenarios `21`–`25` are Part 2 additions (3 cones on one or both sides).

---

### Files Overview

- `src/models.py`: Data classes for `Cone`, `CarPose`, and `Path2D` alias.
- `src/path_planning.py`: **`PathPlanning`** — the main implementation (`generatePath`).
- `src/tester.py`: Matplotlib visualizer that plots cones, the car, heading, and the path.
- `src/scenarios.py`: Pre-built scenarios 1–25 for testing.
- `src/run.py`: CLI entry point to run and visualize a scenario.

---

### Solution Explanation

#### Algorithm Overview

The algorithm is implemented in [`src/path_planning.py`](src/path_planning.py) inside `PathPlanning.generatePath()`. It runs in three sequential phases:

```
Input: car_pose (x, y, yaw) + cones [(x, y, color)]
   ↓
Phase 1: Order cones along track direction
   ↓
Phase 2: Compute centerline waypoints from boundaries
   ↓
Phase 3: Smooth Catmull-Rom spline + arc-length resampling
   ↓
Output: path [(x, y), ...], 28 points at 0.25m spacing, ~7m long
```

---

#### Phase 1 — Ordering Boundary Cones

When multiple cones exist on the same side, they must be ordered progressively along the track (not just sorted by x or y, which would break on curves).

**Approach: greedy nearest-neighbor chain starting from the cone closest to the car.**

This correctly handles straight and curved boundaries. For 1 or 2 cones, the order is trivially determined.

**Orientation correction (cross-product check):**  
After chaining, we verify the boundary direction is consistent with the track layout using the 2D cross product:

```
v = (last_cone - first_cone)
d_other = (closest_opposite_cone - first_cone)
cross = v.x * d_other.y - v.y * d_other.x
```

- If yellow cones (right side) and `cross < 0` → the ordering is flipped → reverse.
- If blue cones (left side) and `cross > 0` → the ordering is flipped → reverse.

This ensures the path direction is always forward relative to both boundaries.

---

#### Phase 2 — Centerline Waypoints

Given ordered yellow and blue cone lists, we compute the track centerline midpoints:

| Available cones | Strategy |
|---|---|
| 0 cones on both sides | Straight line ahead along `yaw` |
| 1 yellow + 1 blue | Midpoint of gate; forward tangent from perpendicular |
| N yellow + 1 blue | Assume lane width from nearest gate pair, project virtual blue cones |
| 1 yellow + N blue | Symmetric to above |
| N yellow + M blue | Resample both polylines to equal length, average midpoints |
| N yellow only | Compute inward normal (left) at each cone, offset by `R=1.0m` |
| N blue only | Compute inward normal (right) at each cone, offset by `R=1.0m` |

**Nominal half-width `R = 1.0 m`** is the assumed half-track width when only one boundary is visible. This is a design assumption — in a real system it would come from track specifications.

---

#### Phase 3 — Smooth Path Generation (Catmull-Rom Spline)

A **Catmull-Rom spline** is interpolated through the computed waypoints. It produces a smooth C¹ continuous curve that passes exactly through every waypoint, with no oscillation between them.

**Key design choices:**
1. **Lead control point**: A point is inserted `0.4–1.0m` ahead of the car along `yaw` to ensure the path starts tangent to the car's current heading.
2. **Forward extension**: If the waypoints don't cover the target path length (7m), the path is extended linearly along the final tangent direction.
3. **Arc-length resampling**: After generating a dense set of spline points, the path is resampled at exactly `step=0.25m` intervals using cumulative arc-length interpolation — ensuring every step is within the `<= 0.5m` constraint.

**Result:** 28 path points, all exactly 0.25m apart, total length ~6.7–6.8m.

---

#### Part 2 — Handling 3 Cones Per Side

The same algorithm handles 3+ cones seamlessly:

- **3 blue cones (left only)**: Greedy chain, compute right-side normals at each cone offset by `R=1.0m`.
- **3 yellow cones (right only)**: Greedy chain, compute left-side normals at each cone offset by `R=1.0m`.
- **3 blue + 1 yellow**: Projects a virtual matching yellow boundary by repeating the gate width offset, then averages midpoints.
- **3 yellow + 2 blue** (or 3+3): Resamples both polylines to the same density and averages midpoints.

New test scenarios 21–25 cover all these configurations. Run them with:

```bash
DISPLAY=:0 python -m src.run --scenario 21   # 3 blue only
DISPLAY=:0 python -m src.run --scenario 22   # 3 yellow only
DISPLAY=:0 python -m src.run --scenario 23   # 3 blue + 1 yellow
DISPLAY=:0 python -m src.run --scenario 24   # 3 yellow + 2 blue
DISPLAY=:0 python -m src.run --scenario 25   # 3 blue + 3 yellow
```

---

### Assumptions Made

1. **Nominal half-track width `R = 1.0 m`**: When only one boundary side is visible, the centerline is offset `1.0m` inward from the visible cones. Real FSAI track width is typically 3–4m, giving a half-width of ~1.5–2m.
2. **All cones belong to the immediately upcoming track section**: We don't filter cones behind the car or far off-track.
3. **The car starts at or near the track entrance**: The heading vector `yaw` is a reliable guide for the initial path direction.
4. **Uniform cone density**: The greedy nearest-neighbor chaining assumes cones are placed reasonably close to each other with no large gaps that would confuse the ordering.

---

### Limitations

1. **No loop closure**: The algorithm generates a local path only (~7m ahead). It does not plan globally around a full track.
2. **Greedy ordering fails for criss-crossed cones**: If cones from both sides interleave spatially (e.g. extreme S-bends with close cones), the nearest-neighbor chain could misorder them.
3. **Fixed half-width assumption**: When only one boundary is visible, the fixed `R=1.0m` offset may not match the actual track width.
4. **No obstacle avoidance**: The path stays between cones but does not account for dynamic obstacles or safety margins.
5. **No velocity or curvature constraints**: The path is geometrically smooth (C¹) but curvature is not bounded — sharp turns near tight gates could exceed vehicle dynamics limits.
6. **No filtering of noise**: Assumes cone positions are perfect (no GPS/LiDAR noise).

---

### Author's Notes

- The approach is deliberately simple: order → midpoint → spline → resample.
- No external libraries beyond `matplotlib` and standard Python `math` are used.
- The Catmull-Rom spline was chosen because it passes through all control points (unlike Bezier) and is easy to implement without scipy.
- The arc-length resampling ensures the step-size constraint is met exactly.
