import math
import numpy as np
from collections import deque

# Must match simulation.py
_MAZE_W = 720
_MAZE_H = 700


def bug2_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt):
    """
    Bug2 algorithm: drive along M-line toward goal; when obstacle hit, follow boundary
    using a left-hand rule until the M-line is re-crossed closer to goal.

    Returns (new_x, new_y, new_angle).
    """
    SPEED           = 80.0
    TURN_RATE       = 2.8
    SENSOR_DIST     = 42.0
    OBSTACLE_MARGIN = 18.0

    if getattr(bug2_navigate, '_state', None) is None:
        bug2_navigate._state = {
            'mode':               'go_to_goal',
            'hit_point':          None,
            'best_dist_on_mline': None,
            'start_x':            car_x,
            'start_y':            car_y,
        }
    state = bug2_navigate._state

    def point_in_obstacle(x, y):
        for obs in obstacles:
            ox, oy, ow, oh = obs
            if (ox - OBSTACLE_MARGIN <= x <= ox + ow + OBSTACLE_MARGIN and
                    oy - OBSTACLE_MARGIN <= y <= oy + oh + OBSTACLE_MARGIN):
                return True
        return False

    def obstacle_ahead(x, y, angle, dist):
        for frac in [0.5, 1.0]:
            tx = x + math.cos(angle) * dist * frac
            ty = y - math.sin(angle) * dist * frac
            if point_in_obstacle(tx, ty):
                return True
        return False

    def dist_to_goal(x, y):
        return math.hypot(goal_x - x, goal_y - y)

    def on_mline(x, y):
        sx, sy = state['start_x'], state['start_y']
        dx, dy = goal_x - sx, goal_y - sy
        length = math.hypot(dx, dy)
        if length < 1:
            return True
        nx, ny = dx / length, dy / length
        px, py = x - sx, y - sy
        cross = abs(px * (-ny) + py * nx)
        dot   = px * nx + py * ny
        return cross < 18.0 and dot > 0

    target_angle = math.atan2(-(goal_y - car_y), goal_x - car_x)

    # ── Go-to-goal ────────────────────────────────────────────────────────────
    if state['mode'] == 'go_to_goal':
        if obstacle_ahead(car_x, car_y, car_angle, SENSOR_DIST):
            state['mode']               = 'follow_boundary'
            state['hit_point']          = (car_x, car_y)
            state['best_dist_on_mline'] = dist_to_goal(car_x, car_y)
        else:
            angle_diff = _angle_diff(target_angle, car_angle)
            new_angle  = car_angle + math.copysign(min(abs(angle_diff), TURN_RATE * dt), angle_diff)
            new_x = car_x + math.cos(new_angle) * SPEED * dt
            new_y = car_y - math.sin(new_angle) * SPEED * dt
            if not point_in_obstacle(new_x, new_y):
                return new_x, new_y, new_angle
            # Blocked: still rotate toward boundary entry, don't freeze
            car_angle = new_angle
            state['mode']               = 'follow_boundary'
            state['hit_point']          = (car_x, car_y)
            state['best_dist_on_mline'] = dist_to_goal(car_x, car_y)

    # ── Left-hand wall follower ───────────────────────────────────────────────
    if state['mode'] == 'follow_boundary':
        left_angle = car_angle + math.pi / 2

        if obstacle_ahead(car_x, car_y, car_angle, SENSOR_DIST):
            new_angle = car_angle - TURN_RATE * dt          # blocked → turn right
        elif not obstacle_ahead(car_x, car_y, left_angle, SENSOR_DIST * 0.7):
            new_angle = car_angle + TURN_RATE * dt * 0.8   # lost left wall → turn left
        else:
            new_angle = car_angle                           # wall on left, clear ahead

        new_x = car_x + math.cos(new_angle) * SPEED * 0.65 * dt
        new_y = car_y - math.sin(new_angle) * SPEED * 0.65 * dt

        if on_mline(new_x, new_y):
            d = dist_to_goal(new_x, new_y)
            if state['best_dist_on_mline'] is None or d < state['best_dist_on_mline'] - 10:
                state['mode']               = 'go_to_goal'
                state['best_dist_on_mline'] = d

        if not point_in_obstacle(new_x, new_y):
            return new_x, new_y, new_angle
        # Still blocked: rotate in place (return new_angle so car can turn)
        return car_x, car_y, new_angle

    return car_x, car_y, car_angle


# ── BFS path planner (shared by Pure Pursuit and Stanley) ─────────────────────

def _bfs_plan(car_x, car_y, goal_x, goal_y, obstacles, obstacle_margin):
    """
    4-directional BFS grid planner with line-of-sight path smoothing.

    4-directional (no diagonals) is critical: diagonal BFS segments can clip
    obstacle margin zones even when both cell centres are clear.

    When the car sits inside an obstacle margin zone the planner searches
    outward for the nearest free cell and starts the path from there.
    """
    CELL = 20

    cols = _MAZE_W // CELL + 2
    rows = _MAZE_H // CELL + 2

    def w2g(x, y):
        return (max(0, min(cols - 1, int(x // CELL))),
                max(0, min(rows - 1, int(y // CELL))))

    def g2w(gx, gy):
        return gx * CELL + CELL / 2, gy * CELL + CELL / 2

    def blocked(gx, gy):
        wx, wy = g2w(gx, gy)
        for ox, oy, ow, oh in obstacles:
            if (ox - obstacle_margin <= wx <= ox + ow + obstacle_margin and
                    oy - obstacle_margin <= wy <= oy + oh + obstacle_margin):
                return True
        return False

    sg = w2g(car_x, car_y)
    gg = w2g(goal_x, goal_y)

    # If start cell is inside a margin zone find the nearest free cell
    if blocked(*sg):
        found = False
        for r in range(1, 8):
            for dgx in range(-r, r + 1):
                for dgy in range(-r, r + 1):
                    if abs(dgx) == r or abs(dgy) == r:
                        cand = (sg[0] + dgx, sg[1] + dgy)
                        if (0 <= cand[0] < cols and 0 <= cand[1] < rows
                                and not blocked(*cand)):
                            sg = cand
                            found = True
                            break
                if found:
                    break
            if found:
                break

    # 4-directional BFS — axis-aligned moves never clip obstacle margins
    queue     = deque([sg])
    came_from = {sg: None}
    while queue:
        curr = queue.popleft()
        if curr == gg:
            break
        cx, cy = curr
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nxt = (cx + dx, cy + dy)
            if (0 <= nxt[0] < cols and 0 <= nxt[1] < rows
                    and nxt not in came_from and not blocked(*nxt)):
                came_from[nxt] = curr
                queue.append(nxt)

    if gg not in came_from:
        return [g2w(*sg), (goal_x, goal_y)]

    cells = []
    node  = gg
    while node is not None:
        cells.append(node)
        node = came_from[node]
    cells.reverse()

    # Start path from the nearest-unblocked grid cell centre so the car
    # steers OUT of any obstacle margin zone as its first move.
    path = [g2w(*cells[0])] + [g2w(*c) for c in cells[1:-1]] + [(goal_x, goal_y)]

    # Mild LOS smoothing — only merge segments shorter than 2 BFS cells so the
    # car stays close to the grid path and can't deviate enough to clip obstacles.
    MAX_SMOOTH = CELL * 2 + 5

    def los_clear(x1, y1, x2, y2):
        if math.hypot(x2 - x1, y2 - y1) > MAX_SMOOTH:
            return False
        steps = max(int(math.hypot(x2 - x1, y2 - y1) / 6), 2)
        for i in range(steps + 1):
            t  = i / steps
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            for ox, oy, ow, oh in obstacles:
                if (ox - obstacle_margin <= px <= ox + ow + obstacle_margin and
                        oy - obstacle_margin <= py <= oy + oh + obstacle_margin):
                    return False
        return True

    smooth = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if los_clear(path[i][0], path[i][1], path[j][0], path[j][1]):
                break
            j -= 1
        smooth.append(path[j])
        i = j

    return smooth


def _advance_waypoints(waypoints, wp_index, car_x, car_y, car_angle, waypoint_reach):
    """
    Advance waypoint index past any waypoints that are:
    - within waypoint_reach pixels, OR
    - clearly behind the car (negative projection on heading).
    """
    fwd_x = math.cos(car_angle)
    fwd_y = -math.sin(car_angle)   # screen coords: up = negative y
    while wp_index < len(waypoints) - 1:
        wpx, wpy = waypoints[wp_index]
        dx   = wpx - car_x
        dy   = wpy - car_y
        dist = math.hypot(dx, dy)
        dot  = dx * fwd_x + dy * fwd_y
        if dist < waypoint_reach or dot < -5:
            wp_index += 1
        else:
            break
    return wp_index


def pure_pursuit_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt):
    """
    Pure Pursuit: BFS-planned waypoint path + lookahead-point steering.

    Returns (new_x, new_y, new_angle).
    """
    SPEED           = 90.0
    LOOKAHEAD       = 28.0   # short — prevents jumping past tight BFS corners
    TURN_RATE       = 3.2
    OBSTACLE_MARGIN = 20.0
    WAYPOINT_REACH  = 18.0

    if getattr(pure_pursuit_navigate, '_state', None) is None:
        pure_pursuit_navigate._state = {
            'waypoints':    None,
            'wp_index':     0,
            'stuck_frames': 0,
        }
    state = pure_pursuit_navigate._state

    if state['waypoints'] is None:
        state['waypoints'] = _bfs_plan(car_x, car_y, goal_x, goal_y,
                                       obstacles, OBSTACLE_MARGIN)
        state['wp_index']   = 0

    waypoints = state['waypoints']
    wp_index  = _advance_waypoints(waypoints, state['wp_index'],
                                   car_x, car_y, car_angle, WAYPOINT_REACH)
    state['wp_index'] = wp_index
    wp_index = min(wp_index, len(waypoints) - 1)

    # Find lookahead point
    lookahead_x, lookahead_y = waypoints[wp_index]
    for i in range(wp_index, len(waypoints)):
        d = math.hypot(waypoints[i][0] - car_x, waypoints[i][1] - car_y)
        if d >= LOOKAHEAD:
            lookahead_x, lookahead_y = waypoints[i]
            break

    target_angle = math.atan2(-(lookahead_y - car_y), lookahead_x - car_x)
    angle_diff   = _angle_diff(target_angle, car_angle)
    new_angle    = car_angle + math.copysign(min(abs(angle_diff), TURN_RATE * dt), angle_diff)
    new_x = car_x + math.cos(new_angle) * SPEED * dt
    new_y = car_y - math.sin(new_angle) * SPEED * dt

    blocked = any(
        ox - OBSTACLE_MARGIN <= new_x <= ox + ow + OBSTACLE_MARGIN and
        oy - OBSTACLE_MARGIN <= new_y <= oy + oh + OBSTACLE_MARGIN
        for ox, oy, ow, oh in obstacles
    )
    if blocked:
        state['stuck_frames'] += 1
        if state['stuck_frames'] > 45:
            state['waypoints']    = None
            state['wp_index']     = 0
            state['stuck_frames'] = 0
        # Return new_angle so the car can rotate in place past the obstacle
        return car_x, car_y, new_angle

    state['stuck_frames'] = 0
    return new_x, new_y, new_angle


def stanley_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt):
    """
    Stanley controller: BFS-planned path + cross-track / heading-error steering.

    Returns (new_x, new_y, new_angle).
    """
    SPEED           = 85.0
    K               = 2.0
    TURN_RATE       = 3.5
    OBSTACLE_MARGIN = 20.0
    WAYPOINT_REACH  = 18.0

    if getattr(stanley_navigate, '_state', None) is None:
        stanley_navigate._state = {
            'waypoints':    None,
            'wp_index':     0,
            'stuck_frames': 0,
        }
    state = stanley_navigate._state

    if state['waypoints'] is None:
        state['waypoints'] = _bfs_plan(car_x, car_y, goal_x, goal_y,
                                       obstacles, OBSTACLE_MARGIN)
        state['wp_index']   = 0

    waypoints = state['waypoints']
    wp_index  = _advance_waypoints(waypoints, state['wp_index'],
                                   car_x, car_y, car_angle, WAYPOINT_REACH)
    state['wp_index'] = wp_index
    wp_index = min(wp_index, len(waypoints) - 1)

    tx, ty = waypoints[wp_index]
    px, py = waypoints[wp_index - 1] if wp_index > 0 else (car_x, car_y)

    seg_dx    = tx - px
    seg_dy    = ty - py
    seg_len   = math.hypot(seg_dx, seg_dy) + 1e-9
    seg_angle = math.atan2(-seg_dy, seg_dx)

    ex = car_x - px
    ey = car_y - py
    cross_track    = ex * (-seg_dy / seg_len) + ey * (seg_dx / seg_len)
    heading_err    = _angle_diff(seg_angle, car_angle)
    cte_correction = math.atan2(K * cross_track, SPEED)
    delta          = math.copysign(
        min(abs(heading_err + cte_correction), TURN_RATE * dt),
        heading_err + cte_correction,
    )

    new_angle = car_angle + delta
    new_x = car_x + math.cos(new_angle) * SPEED * dt
    new_y = car_y - math.sin(new_angle) * SPEED * dt

    blocked = any(
        ox - OBSTACLE_MARGIN <= new_x <= ox + ow + OBSTACLE_MARGIN and
        oy - OBSTACLE_MARGIN <= new_y <= oy + oh + OBSTACLE_MARGIN
        for ox, oy, ow, oh in obstacles
    )
    if blocked:
        state['stuck_frames'] += 1
        if state['stuck_frames'] > 45:
            state['waypoints']    = None
            state['wp_index']     = 0
            state['stuck_frames'] = 0
        # Return new_angle so the car can rotate in place past the obstacle
        return car_x, car_y, new_angle

    state['stuck_frames'] = 0
    return new_x, new_y, new_angle


def potential_field_navigate(car_x, car_y, car_angle, goal_x, goal_y, obstacles, car_width, car_height, dt):
    """
    Potential Field: attractive force toward goal, repulsive forces from obstacles.
    When stuck in a local minimum the car escapes by pushing directly away from the
    nearest obstacle surface, bypassing the margin check until it is truly free.

    Returns (new_x, new_y, new_angle).
    """
    SPEED           = 78.0
    TURN_RATE       = 3.2
    K_ATT           = 1.0
    K_REP           = 9000.0
    D0              = 65.0
    OBSTACLE_MARGIN = 15.0
    ESCAPE_THRESH   = 18

    if getattr(potential_field_navigate, '_state', None) is None:
        potential_field_navigate._state = {'stuck_frames': 0}
    state = potential_field_navigate._state

    # Nearest obstacle surface — used for escape direction
    nearest_dist  = float('inf')
    nearest_esc_x = 1.0
    nearest_esc_y = 0.0

    att_x = K_ATT * (goal_x - car_x)
    att_y = K_ATT * (goal_y - car_y)

    rep_x, rep_y = 0.0, 0.0
    for obs in obstacles:
        ox, oy, ow, oh = obs
        closest_x = max(ox, min(car_x, ox + ow))
        closest_y = max(oy, min(car_y, oy + oh))
        dx   = car_x - closest_x
        dy   = car_y - closest_y
        dist = math.hypot(dx, dy) + 1e-9
        if dist < D0:
            mag   = K_REP * (1.0 / dist - 1.0 / D0) / (dist ** 2)
            rep_x += mag * dx / dist
            rep_y += mag * dy / dist
        if dist < nearest_dist:
            nearest_dist  = dist
            nearest_esc_x = dx / dist
            nearest_esc_y = dy / dist

    # ── Escape mode: push away from nearest obstacle, bypass margin check ─────
    if state['stuck_frames'] >= ESCAPE_THRESH:
        escape_angle = math.atan2(-nearest_esc_y, nearest_esc_x)
        new_angle    = escape_angle
        new_x = car_x + math.cos(new_angle) * SPEED * 3.0 * dt
        new_y = car_y - math.sin(new_angle) * SPEED * 3.0 * dt
        # Reset only once truly free (keep escaping if still blocked)
        still_blocked = any(
            ox - OBSTACLE_MARGIN <= new_x <= ox + ow + OBSTACLE_MARGIN and
            oy - OBSTACLE_MARGIN <= new_y <= oy + oh + OBSTACLE_MARGIN
            for ox, oy, ow, oh in obstacles
        )
        if not still_blocked:
            state['stuck_frames'] = 0
        return new_x, new_y, new_angle

    # ── Normal potential-field move ───────────────────────────────────────────
    force_x = att_x + rep_x
    force_y = att_y + rep_y
    force_mag = math.hypot(force_x, force_y) + 1e-9
    force_x  /= force_mag
    force_y  /= force_mag

    target_angle = math.atan2(-force_y, force_x)
    angle_diff   = _angle_diff(target_angle, car_angle)
    new_angle    = car_angle + math.copysign(min(abs(angle_diff), TURN_RATE * dt), angle_diff)
    new_x = car_x + math.cos(new_angle) * SPEED * dt
    new_y = car_y - math.sin(new_angle) * SPEED * dt

    blocked = any(
        ox - OBSTACLE_MARGIN <= new_x <= ox + ow + OBSTACLE_MARGIN and
        oy - OBSTACLE_MARGIN <= new_y <= oy + oh + OBSTACLE_MARGIN
        for ox, oy, ow, oh in obstacles
    )
    if blocked or math.hypot(new_x - car_x, new_y - car_y) < 0.05:
        state['stuck_frames'] += 1
        return car_x, car_y, car_angle

    state['stuck_frames'] = max(0, state['stuck_frames'] - 1)
    return new_x, new_y, new_angle


def _angle_diff(target, current):
    diff = target - current
    while diff >  math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    return diff


def reset_algorithm_state():
    """Clear all per-algorithm state dicts (called on maze reset)."""
    for fn in [bug2_navigate, pure_pursuit_navigate,
               stanley_navigate, potential_field_navigate]:
        fn._state = None
