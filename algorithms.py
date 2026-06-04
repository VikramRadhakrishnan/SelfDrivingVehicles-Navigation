import math
import numpy as np


def bug2_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt):
    """
    Bug2 algorithm: drive along M-line toward goal; when obstacle hit, follow boundary
    until M-line is re-crossed closer to goal.

    Returns (new_x, new_y, new_angle).
    """
    SPEED = 80.0
    TURN_RATE = 2.5
    SENSOR_DIST = 40.0
    OBSTACLE_MARGIN = 18.0

    state = getattr(bug2_navigate, '_state', None)
    if state is None or state.get('reset'):
        state = {
            'mode': 'go_to_goal',
            'hit_point': None,
            'best_dist_on_mline': None,
            'boundary_dir': 1,
            'reset': False,
        }
        bug2_navigate._state = state

    def point_in_obstacle(x, y):
        for obs in obstacles:
            ox, oy, ow, oh = obs
            if (ox - OBSTACLE_MARGIN <= x <= ox + ow + OBSTACLE_MARGIN and
                    oy - OBSTACLE_MARGIN <= y <= oy + oh + OBSTACLE_MARGIN):
                return True
        return False

    def obstacle_ahead(x, y, angle, dist):
        for step in [dist * 0.5, dist]:
            tx = x + math.cos(angle) * step
            ty = y - math.sin(angle) * step
            if point_in_obstacle(tx, ty):
                return True
        return False

    def dist_to_goal(x, y):
        return math.hypot(goal_x - x, goal_y - y)

    def on_mline(x, y):
        dx = goal_x - (state.get('start_x', x))
        dy = goal_y - (state.get('start_y', y))
        length = math.hypot(dx, dy)
        if length < 1:
            return True
        nx, ny = dx / length, dy / length
        px = x - state.get('start_x', x)
        py = y - state.get('start_y', y)
        cross = abs(px * (-ny) + py * nx)
        dot = px * nx + py * ny
        return cross < 15.0 and dot > 0

    if 'start_x' not in state:
        state['start_x'] = car_x
        state['start_y'] = car_y

    target_angle = math.atan2(-(goal_y - car_y), goal_x - car_x)

    if state['mode'] == 'go_to_goal':
        if obstacle_ahead(car_x, car_y, car_angle, SENSOR_DIST):
            state['mode'] = 'follow_boundary'
            state['hit_point'] = (car_x, car_y)
            state['best_dist_on_mline'] = dist_to_goal(car_x, car_y)
        else:
            angle_diff = _angle_diff(target_angle, car_angle)
            new_angle = car_angle + math.copysign(min(abs(angle_diff), TURN_RATE * dt), angle_diff)
            new_x = car_x + math.cos(new_angle) * SPEED * dt
            new_y = car_y - math.sin(new_angle) * SPEED * dt
            if not point_in_obstacle(new_x, new_y):
                return new_x, new_y, new_angle
            return car_x, car_y, car_angle

    if state['mode'] == 'follow_boundary':
        wall_angle = car_angle + state['boundary_dir'] * math.pi / 2
        if not obstacle_ahead(car_x, car_y, wall_angle, SENSOR_DIST * 0.6):
            state['boundary_dir'] *= -1

        if obstacle_ahead(car_x, car_y, car_angle, SENSOR_DIST):
            new_angle = car_angle - state['boundary_dir'] * TURN_RATE * dt
        else:
            new_angle = car_angle + state['boundary_dir'] * TURN_RATE * dt * 0.5

        new_x = car_x + math.cos(new_angle) * SPEED * 0.6 * dt
        new_y = car_y - math.sin(new_angle) * SPEED * 0.6 * dt

        if on_mline(new_x, new_y):
            d = dist_to_goal(new_x, new_y)
            if state['best_dist_on_mline'] is None or d < state['best_dist_on_mline'] - 10:
                state['mode'] = 'go_to_goal'
                state['best_dist_on_mline'] = d

        if not point_in_obstacle(new_x, new_y):
            return new_x, new_y, new_angle
        return car_x, car_y, car_angle

    return car_x, car_y, car_angle


def pure_pursuit_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt):
    """
    Pure Pursuit: follow a series of waypoints computed around obstacles using a
    lookahead point on the path.

    Returns (new_x, new_y, new_angle).
    """
    SPEED = 90.0
    LOOKAHEAD = 60.0
    TURN_RATE = 3.0
    OBSTACLE_MARGIN = 20.0
    WAYPOINT_REACH = 30.0

    state = getattr(pure_pursuit_navigate, '_state', None)
    if state is None or state.get('reset'):
        state = {
            'waypoints': None,
            'wp_index': 0,
            'reset': False,
        }
        pure_pursuit_navigate._state = state

    def point_clear(x, y):
        for obs in obstacles:
            ox, oy, ow, oh = obs
            if (ox - OBSTACLE_MARGIN <= x <= ox + ow + OBSTACLE_MARGIN and
                    oy - OBSTACLE_MARGIN <= y <= oy + oh + OBSTACLE_MARGIN):
                return False
        return True

    def path_clear(x1, y1, x2, y2, steps=10):
        for i in range(steps + 1):
            t = i / steps
            if not point_clear(x1 + t * (x2 - x1), y1 + t * (y2 - y1)):
                return False
        return True

    def build_waypoints():
        waypoints = [(car_x, car_y)]
        current = (car_x, car_y)
        target = (goal_x, goal_y)
        max_iter = 200
        step = 50.0
        for _ in range(max_iter):
            cx, cy = current
            gx, gy = target
            if math.hypot(gx - cx, gy - cy) < step:
                waypoints.append(target)
                break
            base_angle = math.atan2(-(gy - cy), gx - cx)
            placed = False
            for da in [0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6, math.pi]:
                a = base_angle + da
                nx = cx + math.cos(a) * step
                ny = cy - math.sin(a) * step
                if point_clear(nx, ny) and path_clear(cx, cy, nx, ny):
                    waypoints.append((nx, ny))
                    current = (nx, ny)
                    placed = True
                    break
            if not placed:
                waypoints.append(target)
                break
        return waypoints

    if state['waypoints'] is None:
        state['waypoints'] = build_waypoints()
        state['wp_index'] = 0

    waypoints = state['waypoints']
    wp_index = state['wp_index']

    while wp_index < len(waypoints) - 1:
        wpx, wpy = waypoints[wp_index]
        if math.hypot(wpx - car_x, wpy - car_y) < WAYPOINT_REACH:
            wp_index += 1
            state['wp_index'] = wp_index
        else:
            break

    if wp_index >= len(waypoints):
        wp_index = len(waypoints) - 1

    lookahead_x, lookahead_y = waypoints[min(wp_index, len(waypoints) - 1)]

    for i in range(wp_index, len(waypoints)):
        if math.hypot(waypoints[i][0] - car_x, waypoints[i][1] - car_y) >= LOOKAHEAD:
            lookahead_x, lookahead_y = waypoints[i]
            break

    target_angle = math.atan2(-(lookahead_y - car_y), lookahead_x - car_x)
    angle_diff = _angle_diff(target_angle, car_angle)
    new_angle = car_angle + math.copysign(min(abs(angle_diff), TURN_RATE * dt), angle_diff)
    new_x = car_x + math.cos(new_angle) * SPEED * dt
    new_y = car_y - math.sin(new_angle) * SPEED * dt

    for obs in obstacles:
        ox, oy, ow, oh = obs
        if (ox - OBSTACLE_MARGIN <= new_x <= ox + ow + OBSTACLE_MARGIN and
                oy - OBSTACLE_MARGIN <= new_y <= oy + oh + OBSTACLE_MARGIN):
            state['waypoints'] = None
            return car_x, car_y, car_angle

    return new_x, new_y, new_angle


def stanley_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt):
    """
    Stanley controller: minimize cross-track error and heading error to a reference path.

    Returns (new_x, new_y, new_angle).
    """
    SPEED = 85.0
    K = 2.0
    TURN_RATE = 3.5
    OBSTACLE_MARGIN = 20.0
    WAYPOINT_REACH = 25.0

    state = getattr(stanley_navigate, '_state', None)
    if state is None or state.get('reset'):
        state = {
            'waypoints': None,
            'wp_index': 0,
            'reset': False,
        }
        stanley_navigate._state = state

    def point_clear(x, y):
        for obs in obstacles:
            ox, oy, ow, oh = obs
            if (ox - OBSTACLE_MARGIN <= x <= ox + ow + OBSTACLE_MARGIN and
                    oy - OBSTACLE_MARGIN <= y <= oy + oh + OBSTACLE_MARGIN):
                return False
        return True

    def path_clear(x1, y1, x2, y2, steps=10):
        for i in range(steps + 1):
            t = i / steps
            if not point_clear(x1 + t * (x2 - x1), y1 + t * (y2 - y1)):
                return False
        return True

    def build_waypoints():
        waypoints = [(car_x, car_y)]
        current = (car_x, car_y)
        target = (goal_x, goal_y)
        step = 50.0
        for _ in range(200):
            cx, cy = current
            gx, gy = target
            if math.hypot(gx - cx, gy - cy) < step:
                waypoints.append(target)
                break
            base_angle = math.atan2(-(gy - cy), gx - cx)
            placed = False
            for da in [0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6, math.pi]:
                a = base_angle + da
                nx = cx + math.cos(a) * step
                ny = cy - math.sin(a) * step
                if point_clear(nx, ny) and path_clear(cx, cy, nx, ny):
                    waypoints.append((nx, ny))
                    current = (nx, ny)
                    placed = True
                    break
            if not placed:
                waypoints.append(target)
                break
        return waypoints

    if state['waypoints'] is None:
        state['waypoints'] = build_waypoints()
        state['wp_index'] = 0

    waypoints = state['waypoints']
    wp_index = state['wp_index']

    while wp_index < len(waypoints) - 1:
        wpx, wpy = waypoints[wp_index]
        if math.hypot(wpx - car_x, wpy - car_y) < WAYPOINT_REACH:
            wp_index += 1
            state['wp_index'] = wp_index
        else:
            break

    wp_index = min(wp_index, len(waypoints) - 1)
    tx, ty = waypoints[wp_index]

    if wp_index > 0:
        px, py = waypoints[wp_index - 1]
    else:
        px, py = car_x, car_y

    seg_dx = tx - px
    seg_dy = ty - py
    seg_len = math.hypot(seg_dx, seg_dy) + 1e-9
    seg_angle = math.atan2(-seg_dy, seg_dx)

    ex = car_x - px
    ey = car_y - py
    cross_track = (ex * (-seg_dy / seg_len) + ey * (seg_dx / seg_len))

    heading_err = _angle_diff(seg_angle, car_angle)
    cte_correction = math.atan2(K * cross_track, SPEED)
    delta = heading_err + cte_correction

    max_delta = TURN_RATE * dt
    delta = math.copysign(min(abs(delta), max_delta), delta)

    new_angle = car_angle + delta
    new_x = car_x + math.cos(new_angle) * SPEED * dt
    new_y = car_y - math.sin(new_angle) * SPEED * dt

    for obs in obstacles:
        ox, oy, ow, oh = obs
        if (ox - OBSTACLE_MARGIN <= new_x <= ox + ow + OBSTACLE_MARGIN and
                oy - OBSTACLE_MARGIN <= new_y <= oy + oh + OBSTACLE_MARGIN):
            state['waypoints'] = None
            return car_x, car_y, car_angle

    return new_x, new_y, new_angle


def potential_field_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt):
    """
    Potential Field: attractive force toward goal, repulsive forces from obstacles.
    Gradient descent on combined potential.

    Returns (new_x, new_y, new_angle).
    """
    SPEED = 75.0
    TURN_RATE = 3.0
    K_ATT = 1.0
    K_REP = 8000.0
    D0 = 60.0
    OBSTACLE_MARGIN = 15.0

    att_x = K_ATT * (goal_x - car_x)
    att_y = K_ATT * (goal_y - car_y)

    rep_x, rep_y = 0.0, 0.0
    for obs in obstacles:
        ox, oy, ow, oh = obs
        cx_obs = ox + ow / 2
        cy_obs = oy + oh / 2

        closest_x = max(ox, min(car_x, ox + ow))
        closest_y = max(oy, min(car_y, oy + oh))

        dx = car_x - closest_x
        dy = car_y - closest_y
        dist = math.hypot(dx, dy) + 1e-9

        if dist < D0:
            mag = K_REP * (1.0 / dist - 1.0 / D0) / (dist ** 2)
            rep_x += mag * dx / dist
            rep_y += mag * dy / dist

    force_x = att_x + rep_x
    force_y = att_y + rep_y

    force_mag = math.hypot(force_x, force_y) + 1e-9
    force_x /= force_mag
    force_y /= force_mag

    target_angle = math.atan2(-force_y, force_x)
    angle_diff = _angle_diff(target_angle, car_angle)
    new_angle = car_angle + math.copysign(min(abs(angle_diff), TURN_RATE * dt), angle_diff)

    new_x = car_x + math.cos(new_angle) * SPEED * dt
    new_y = car_y - math.sin(new_angle) * SPEED * dt

    for obs in obstacles:
        ox, oy, ow, oh = obs
        if (ox - OBSTACLE_MARGIN <= new_x <= ox + ow + OBSTACLE_MARGIN and
                oy - OBSTACLE_MARGIN <= new_y <= oy + oh + OBSTACLE_MARGIN):
            perturb = 0.3
            new_angle = car_angle + perturb * (1 if angle_diff >= 0 else -1)
            new_x = car_x + math.cos(new_angle) * SPEED * 0.5 * dt
            new_y = car_y - math.sin(new_angle) * SPEED * 0.5 * dt
            for obs2 in obstacles:
                ox2, oy2, ow2, oh2 = obs2
                if (ox2 - OBSTACLE_MARGIN <= new_x <= ox2 + ow2 + OBSTACLE_MARGIN and
                        oy2 - OBSTACLE_MARGIN <= new_y <= oy2 + oh2 + OBSTACLE_MARGIN):
                    return car_x, car_y, car_angle
            break

    return new_x, new_y, new_angle


def _angle_diff(target, current):
    diff = target - current
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def reset_algorithm_state():
    for fn in [bug2_navigate, pure_pursuit_navigate, stanley_navigate, potential_field_navigate]:
        if hasattr(fn, '_state'):
            fn._state['reset'] = True
            fn._state = None
