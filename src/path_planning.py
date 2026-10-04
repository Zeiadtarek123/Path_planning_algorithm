from __future__ import annotations

import math
from typing import List, Tuple

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner for Formula Student / FSAI track driving.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a sequence of path points (x, y) in world coordinates.

        Requirements and constraints:
        - Yellow cones (color == 0) are on the RIGHT side of the track.
        - Blue cones (color == 1) are on the LEFT side of the track.
        - Supports 0, 1, 2, 3, or more cones per boundary.
        - Path length is between 5 and 10 meters (target ~7.0 m).
        - Step size is <= 0.5 meters (sampled at exact 0.25 m).
        - Smooth kinematic transition starting tangent to current car heading.
        """
        car_x, car_y, yaw = self.car_pose.x, self.car_pose.y, self.car_pose.yaw
        car_heading = (math.cos(yaw), math.sin(yaw))

        # Separate cones by color
        yellows = [c for c in self.cones if c.color == 0]
        blues = [c for c in self.cones if c.color == 1]

        # Case 0: No cones visible -> drive straight ahead along yaw
        if not yellows and not blues:
            return self._straight_path(car_x, car_y, car_heading, length=7.0, step=0.25)

        # Step 1: Order cones along track progression
        yellows_ordered = self._order_boundary(yellows, is_yellow=True, other_cones=blues, car_x=car_x, car_y=car_y)
        blues_ordered = self._order_boundary(blues, is_yellow=False, other_cones=yellows, car_x=car_x, car_y=car_y)

        # Step 2: Compute centerline waypoints
        waypoints = self._compute_centerline_waypoints(
            yellows=yellows_ordered,
            blues=blues_ordered,
            car_x=car_x,
            car_y=car_y,
            car_heading=car_heading,
            nominal_half_width=1.0,
        )

        # Step 3: Generate smooth spline path through waypoints
        path = self._generate_smooth_path(car_x, car_y, car_heading, waypoints, target_length=7.0, step=0.25)
        return path

    def _straight_path(self, x0: float, y0: float, heading: Tuple[float, float], length: float, step: float) -> Path2D:
        """Produce a straight-line path starting from (x0, y0) along heading."""
        num_steps = int(length / step)
        return [(x0 + heading[0] * i * step, y0 + heading[1] * i * step) for i in range(1, num_steps + 1)]

    def _order_boundary(
        self,
        cones: List[Cone],
        is_yellow: bool,
        other_cones: List[Cone],
        car_x: float,
        car_y: float,
    ) -> List[Cone]:
        """Order cones along track progression direction."""
        if len(cones) <= 1:
            return cones

        # Start from the cone closest to the car, greedily chain nearest neighbors
        start_c = min(cones, key=lambda c: math.hypot(c.x - car_x, c.y - car_y))
        ordered = [start_c]
        remaining = [c for c in cones if c != start_c]
        while remaining:
            curr = ordered[-1]
            next_c = min(remaining, key=lambda c: math.hypot(c.x - curr.x, c.y - curr.y))
            ordered.append(next_c)
            remaining.remove(next_c)

        # Verify track orientation using 2D cross product with opposite boundary:
        # Yellow on right (is_yellow=True): blue cones must be on LEFT (cross > 0)
        # Blue on left (is_yellow=False): yellow cones must be on RIGHT (cross < 0)
        if other_cones:
            v = (ordered[-1].x - ordered[0].x, ordered[-1].y - ordered[0].y)
            closest_other = min(other_cones, key=lambda o: min(math.hypot(o.x - c.x, o.y - c.y) for c in ordered))
            d_other = (closest_other.x - ordered[0].x, closest_other.y - ordered[0].y)
            cross = v[0] * d_other[1] - v[1] * d_other[0]
            if is_yellow and cross < 0:
                ordered.reverse()
            elif not is_yellow and cross > 0:
                ordered.reverse()

        return ordered

    def _compute_centerline_waypoints(
        self,
        yellows: List[Cone],
        blues: List[Cone],
        car_x: float,
        car_y: float,
        car_heading: Tuple[float, float],
        nominal_half_width: float,
    ) -> List[Tuple[float, float]]:
        """Compute centerline waypoints from detected boundary cones."""
        R = nominal_half_width

        # Case A: Both boundaries are observed
        if yellows and blues:
            # Subcase A1: Exactly 1 cone on each side (single track gate)
            if len(yellows) == 1 and len(blues) == 1:
                y0, b0 = yellows[0], blues[0]
                mid = ((y0.x + b0.x) / 2.0, (y0.y + b0.y) / 2.0)
                # Gate vector from yellow (right) to blue (left)
                vx, vy = b0.x - y0.x, b0.y - y0.y
                d = math.hypot(vx, vy)
                # Perpendicular forward tangent (rotated 90 deg clockwise)
                tx, ty = (vy / d, -vx / d) if d > 1e-4 else car_heading
                # Align tangent direction away from car
                to_mid = (mid[0] - car_x, mid[1] - car_y)
                if tx * to_mid[0] + ty * to_mid[1] < 0:
                    tx, ty = -tx, -ty
                return [mid, (mid[0] + tx * 2.0, mid[1] + ty * 2.0)]

            # Subcase A2: Multiple yellow cones, 1 blue cone (project missing blue boundary)
            if len(yellows) >= 2 and len(blues) == 1:
                b0 = blues[0]
                closest_y = min(yellows, key=lambda y: math.hypot(y.x - b0.x, y.y - b0.y))
                off_x, off_y = b0.x - closest_y.x, b0.y - closest_y.y
                return [((y.x + (y.x + off_x)) / 2.0, (y.y + (y.y + off_y)) / 2.0) for y in yellows]

            # Subcase A3: Multiple blue cones, 1 yellow cone (project missing yellow boundary)
            if len(blues) >= 2 and len(yellows) == 1:
                y0 = yellows[0]
                closest_b = min(blues, key=lambda b: math.hypot(b.x - y0.x, b.y - y0.y))
                off_x, off_y = y0.x - closest_b.x, y0.y - closest_b.y
                return [((b.x + (b.x + off_x)) / 2.0, (b.y + (b.y + off_y)) / 2.0) for b in blues]

            # Subcase A4: Multiple cones on both sides (e.g. 2 & 2, 3 & 3, 3 & 2)
            num_samples = max(len(yellows), len(blues), 4)
            y_pts = self._resample_polyline([(y.x, y.y) for y in yellows], num_samples)
            b_pts = self._resample_polyline([(b.x, b.y) for b in blues], num_samples)
            return [((y_pts[i][0] + b_pts[i][0]) / 2.0, (y_pts[i][1] + b_pts[i][1]) / 2.0) for i in range(num_samples)]

        # Case B: Only one boundary is observed
        cones = yellows if yellows else blues
        is_yellow = bool(yellows)
        pts = [(c.x, c.y) for c in cones]

        # Single cone visible
        if len(pts) == 1:
            cx, cy = pts[0]
            to_cone = (cx - car_x, cy - car_y)
            dist = math.hypot(to_cone[0], to_cone[1])
            if dist > 1e-4:
                fx = 0.5 * car_heading[0] + 0.5 * (to_cone[0] / dist)
                fy = 0.5 * car_heading[1] + 0.5 * (to_cone[1] / dist)
                fn = math.hypot(fx, fy)
                fx, fy = (fx / fn, fy / fn) if fn > 1e-4 else car_heading
            else:
                fx, fy = car_heading

            # Offset inward to centerline (yellow is right -> left normal; blue is left -> right normal)
            ox, oy = (-fy * R, fx * R) if is_yellow else (fy * R, -fx * R)
            mid = (cx + ox, cy + oy)
            return [mid, (mid[0] + fx * 2.0, mid[1] + fy * 2.0)]

        # Multiple cones on single boundary (2, 3, or more)
        mids = []
        for i in range(len(pts)):
            if i < len(pts) - 1:
                vx = pts[i + 1][0] - pts[i][0]
                vy = pts[i + 1][1] - pts[i][1]
            else:
                vx = pts[i][0] - pts[i - 1][0]
                vy = pts[i][1] - pts[i - 1][1]
            d = math.hypot(vx, vy)
            ux, uy = (vx / d, vy / d) if d > 1e-4 else car_heading

            # Offset inward into track interior
            nx, ny = (-uy, ux) if is_yellow else (uy, -ux)
            mids.append((pts[i][0] + nx * R, pts[i][1] + ny * R))

        return mids

    def _resample_polyline(self, pts: List[Tuple[float, float]], num_samples: int) -> List[Tuple[float, float]]:
        """Uniformly resample a polyline into num_samples points along its length."""
        if len(pts) == 1:
            return pts * num_samples
        cum_dists = [0.0]
        for i in range(len(pts) - 1):
            d = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            cum_dists.append(cum_dists[-1] + d)
        total_len = cum_dists[-1]
        if total_len < 1e-6:
            return [pts[0]] * num_samples

        resampled = []
        for j in range(num_samples):
            target_d = (j / (num_samples - 1)) * total_len
            for k in range(len(cum_dists) - 1):
                if cum_dists[k] <= target_d <= cum_dists[k + 1]:
                    seg_len = cum_dists[k + 1] - cum_dists[k]
                    t = (target_d - cum_dists[k]) / seg_len if seg_len > 1e-6 else 0.0
                    rx = pts[k][0] + t * (pts[k + 1][0] - pts[k][0])
                    ry = pts[k][1] + t * (pts[k + 1][1] - pts[k][1])
                    resampled.append((rx, ry))
                    break
        return resampled

    def _generate_smooth_path(
        self,
        car_x: float,
        car_y: float,
        car_heading: Tuple[float, float],
        waypoints: List[Tuple[float, float]],
        target_length: float = 7.0,
        step: float = 0.25,
    ) -> Path2D:
        """Fit a smooth Catmull-Rom spline from the car pose through waypoints."""
        if not waypoints:
            return self._straight_path(car_x, car_y, car_heading, target_length, step)

        # Place a lead control point ahead of the car along car_heading
        # to ensure initial path tangency matches car heading
        d_to_first = math.hypot(waypoints[0][0] - car_x, waypoints[0][1] - car_y)
        lead_dist = max(0.4, min(1.0, d_to_first * 0.4))
        p_lead = (car_x + car_heading[0] * lead_dist, car_y + car_heading[1] * lead_dist)

        knots = [(car_x, car_y), p_lead] + waypoints

        # Filter out knots that are too close to each other
        filtered = [knots[0]]
        for pt in knots[1:]:
            if math.hypot(pt[0] - filtered[-1][0], pt[1] - filtered[-1][1]) > 0.15:
                filtered.append(pt)
        knots = filtered

        # Extend knots forward along the exit tangent if shorter than target length
        knot_len = sum(math.hypot(knots[i + 1][0] - knots[i][0], knots[i + 1][1] - knots[i][1]) for i in range(len(knots) - 1))
        if knot_len < target_length + 1.0:
            last_dx = knots[-1][0] - knots[-2][0]
            last_dy = knots[-1][1] - knots[-2][1]
            d = math.hypot(last_dx, last_dy)
            ux, uy = (last_dx / d, last_dy / d) if d > 1e-4 else car_heading
            rem_len = target_length + 2.0 - knot_len
            knots.append((knots[-1][0] + ux * rem_len, knots[-1][1] + uy * rem_len))

        # Sample Catmull-Rom spline with constant step size
        return self._catmull_rom_sample(knots, step=step, target_length=target_length)

    def _catmull_rom_sample(
        self,
        knots: List[Tuple[float, float]],
        step: float,
        target_length: float,
    ) -> Path2D:
        """Sample a centripetal/uniform Catmull-Rom spline with exact arc-length step spacing."""
        if len(knots) < 2:
            return []

        # Virtual boundary control points
        p_start = (2 * knots[0][0] - knots[1][0], 2 * knots[0][1] - knots[1][1])
        p_end = (2 * knots[-1][0] - knots[-2][0], 2 * knots[-1][1] - knots[-2][1])
        ext_knots = [p_start] + knots + [p_end]

        dense_pts = []
        samples_per_seg = 30
        for i in range(len(ext_knots) - 3):
            p0, p1, p2, p3 = ext_knots[i], ext_knots[i + 1], ext_knots[i + 2], ext_knots[i + 3]
            for t_i in range(samples_per_seg):
                t = t_i / float(samples_per_seg)
                t2 = t * t
                t3 = t2 * t
                x = 0.5 * (
                    (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
                    + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
                    + (-p0[0] + p2[0]) * t
                    + 2 * p1[0]
                )
                y = 0.5 * (
                    (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
                    + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
                    + (-p0[1] + p2[1]) * t
                    + 2 * p1[1]
                )
                dense_pts.append((x, y))
        dense_pts.append(knots[-1])

        # Precise arc-length resampling with EXACT step size
        cum_dists = [0.0]
        for i in range(len(dense_pts) - 1):
            d = math.hypot(dense_pts[i + 1][0] - dense_pts[i][0], dense_pts[i + 1][1] - dense_pts[i][1])
            cum_dists.append(cum_dists[-1] + d)

        total_curve_len = cum_dists[-1]
        num_points = int(target_length / step)
        path: Path2D = []

        curr_idx = 0
        for pt_idx in range(1, num_points + 1):
            target_d = pt_idx * step
            if target_d > total_curve_len:
                break
            while curr_idx < len(cum_dists) - 2 and cum_dists[curr_idx + 1] < target_d:
                curr_idx += 1

            d0 = cum_dists[curr_idx]
            d1 = cum_dists[curr_idx + 1]
            seg_len = d1 - d0
            t = (target_d - d0) / seg_len if seg_len > 1e-6 else 0.0
            px = dense_pts[curr_idx][0] + t * (dense_pts[curr_idx + 1][0] - dense_pts[curr_idx][0])
            py = dense_pts[curr_idx][1] + t * (dense_pts[curr_idx + 1][1] - dense_pts[curr_idx][1])
            path.append((px, py))

        return path
