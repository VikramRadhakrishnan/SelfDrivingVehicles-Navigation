import pygame
import sys
import math
import random
from algorithms import (
    bug2_navigate,
    pure_pursuit_navigate,
    stanley_navigate,
    potential_field_navigate,
    reset_algorithm_state,
)

# ── Window & layout ───────────────────────────────────────────────────────────
SCREEN_W, SCREEN_H = 900, 700
SIDEBAR_W = 180
MAZE_W = SCREEN_W - SIDEBAR_W
MAZE_H = SCREEN_H

# ── Colors ────────────────────────────────────────────────────────────────────
WHITE      = (255, 255, 255)
BLACK      = (0,   0,   0)
GRAY       = (180, 180, 180)
DARK_GRAY  = (80,  80,  80)
LIGHT_GRAY = (220, 220, 220)
RED        = (220, 50,  50)
GREEN      = (50,  180, 50)
BLUE       = (50,  100, 220)
DARK_BLUE  = (30,  60,  150)
YELLOW     = (240, 200, 0)
ORANGE     = (240, 130, 0)
BG_COLOR   = (240, 240, 245)
MAZE_BG    = (250, 250, 255)
OBS_COLOR  = (70,  80,  100)
OBS_BORDER = (40,  50,  70)
GOAL_COLOR = (50,  200, 80)
START_COLOR= (50,  100, 220)

# ── Car geometry ──────────────────────────────────────────────────────────────
CAR_W, CAR_H = 24, 14

# ── Start / goal positions ────────────────────────────────────────────────────
MARGIN = 40
START_X = MARGIN
START_Y = MAZE_H - MARGIN
GOAL_X  = MAZE_W - MARGIN
GOAL_Y  = MARGIN

# ── Obstacle generation ───────────────────────────────────────────────────────
OBS_COUNT  = 9
OBS_MIN_W  = 30
OBS_MAX_W  = 65
OBS_MIN_H  = 22
OBS_MAX_H  = 50
CLEAR_ZONE = 65


def generate_obstacles():
    obstacles = []
    for _ in range(OBS_COUNT * 10):
        if len(obstacles) >= OBS_COUNT:
            break
        w = random.randint(OBS_MIN_W, OBS_MAX_W)
        h = random.randint(OBS_MIN_H, OBS_MAX_H)
        x = random.randint(10, MAZE_W - w - 10)
        y = random.randint(10, MAZE_H - h - 10)

        if (x < START_X + CLEAR_ZONE and y > START_Y - CLEAR_ZONE):
            continue
        if (x + w > GOAL_X - CLEAR_ZONE and y < GOAL_Y + CLEAR_ZONE):
            continue

        overlap = False
        for ox, oy, ow, oh in obstacles:
            if not (x + w + 8 < ox or x > ox + ow + 8 or
                    y + h + 8 < oy or y > oy + oh + 8):
                overlap = True
                break
        if not overlap:
            obstacles.append((x, y, w, h))
    return obstacles


# ── Dropdown widget ───────────────────────────────────────────────────────────
class Dropdown:
    def __init__(self, x, y, w, h, options, font):
        self.rect   = pygame.Rect(x, y, w, h)
        self.options = options
        self.selected = 0
        self.open   = False
        self.font   = font
        self.item_h = h

    def draw(self, surface):
        color = BLUE if self.open else DARK_GRAY
        pygame.draw.rect(surface, color, self.rect, border_radius=6)
        pygame.draw.rect(surface, WHITE, self.rect, 2, border_radius=6)
        label = self.font.render(self.options[self.selected], True, WHITE)
        lx = self.rect.x + (self.rect.w - label.get_width()) // 2
        ly = self.rect.y + (self.rect.h - label.get_height()) // 2
        surface.blit(label, (lx, ly))

        arrow = "▲" if self.open else "▼"
        arr = self.font.render(arrow, True, WHITE)
        surface.blit(arr, (self.rect.right - arr.get_width() - 6,
                           self.rect.y + (self.rect.h - arr.get_height()) // 2))

        if self.open:
            for i, opt in enumerate(self.options):
                r = pygame.Rect(self.rect.x,
                                self.rect.bottom + i * self.item_h,
                                self.rect.w, self.item_h)
                bg = BLUE if i == self.selected else DARK_GRAY
                pygame.draw.rect(surface, bg, r, border_radius=4)
                pygame.draw.rect(surface, WHITE, r, 1, border_radius=4)
                txt = self.font.render(opt, True, WHITE)
                surface.blit(txt, (r.x + (r.w - txt.get_width()) // 2,
                                   r.y + (r.h - txt.get_height()) // 2))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.open = not self.open
                return False
            if self.open:
                for i in range(len(self.options)):
                    r = pygame.Rect(self.rect.x,
                                    self.rect.bottom + i * self.item_h,
                                    self.rect.w, self.item_h)
                    if r.collidepoint(event.pos):
                        changed = self.selected != i
                        self.selected = i
                        self.open = False
                        return changed
                self.open = False
        return False


# ── Button widget ─────────────────────────────────────────────────────────────
class Button:
    def __init__(self, x, y, w, h, label, font, color=RED):
        self.rect  = pygame.Rect(x, y, w, h)
        self.label = label
        self.font  = font
        self.color = color
        self.hover = False

    def draw(self, surface):
        c = tuple(min(255, v + 30) for v in self.color) if self.hover else self.color
        pygame.draw.rect(surface, c, self.rect, border_radius=8)
        pygame.draw.rect(surface, WHITE, self.rect, 2, border_radius=8)
        txt = self.font.render(self.label, True, WHITE)
        surface.blit(txt, (self.rect.x + (self.rect.w - txt.get_width()) // 2,
                           self.rect.y + (self.rect.h - txt.get_height()) // 2))

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hover = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                return True
        return False


# ── Car drawing ───────────────────────────────────────────────────────────────
def draw_car(surface, x, y, angle, color=BLUE):
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    hw, hh = CAR_W / 2, CAR_H / 2
    corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
    rotated = [(x + lx * cos_a - ly * (-sin_a),
                y + lx * (-sin_a) + ly * (-cos_a)) for lx, ly in corners]
    pygame.draw.polygon(surface, color, rotated)
    pygame.draw.polygon(surface, WHITE, rotated, 2)

    # Front indicator
    fx = x + cos_a * hw
    fy = y - sin_a * hw
    pygame.draw.circle(surface, YELLOW, (int(fx), int(fy)), 4)


# ── Trail ─────────────────────────────────────────────────────────────────────
MAX_TRAIL = 600


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Self-Driving Car — Maze Navigation")
    clock = pygame.time.Clock()

    font_sm = pygame.font.SysFont("segoeui", 13)
    font_md = pygame.font.SysFont("segoeui", 15, bold=True)
    font_lg = pygame.font.SysFont("segoeui", 20, bold=True)
    font_title = pygame.font.SysFont("segoeui", 17, bold=True)

    ALGO_NAMES   = ["Bug2", "Pure Pursuit", "Stanley", "Potential Field"]
    ALGO_FUNCS   = [bug2_navigate, pure_pursuit_navigate,
                    stanley_navigate, potential_field_navigate]

    sidebar_x = MAZE_W
    pad = 12
    btn_w = SIDEBAR_W - 2 * pad

    dropdown = Dropdown(sidebar_x + pad, 80, btn_w, 28, ALGO_NAMES, font_sm)
    reset_btn = Button(sidebar_x + pad, 232, btn_w, 32, "Reset Maze", font_md, RED)

    obstacles = generate_obstacles()

    def reset(new_obstacles=True):
        nonlocal obstacles, car_x, car_y, car_angle, trail, reached, stuck_timer
        if new_obstacles:
            obstacles = generate_obstacles()
        car_x, car_y = float(START_X), float(START_Y)
        car_angle = math.pi / 4
        trail = []
        reached = False
        stuck_timer = 0.0
        reset_algorithm_state()

    car_x, car_y = float(START_X), float(START_Y)
    car_angle = math.pi / 4
    trail = []
    reached = False
    stuck_timer = 0.0
    prev_pos = (car_x, car_y)
    status_msg = ""
    status_color = BLACK

    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        dt = min(dt, 0.05)

        # ── Events ────────────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

            if dropdown.handle_event(event):
                reset(new_obstacles=False)

            if reset_btn.handle_event(event):
                reset()

        # ── Navigation ────────────────────────────────────────────────────────
        goal_reached = math.hypot(GOAL_X - car_x, GOAL_Y - car_y) < 25

        if not goal_reached and not reached:
            algo = ALGO_FUNCS[dropdown.selected]
            new_x, new_y, new_angle = algo(
                car_x, car_y, car_angle,
                GOAL_X, GOAL_Y,
                obstacles,
                CAR_W, CAR_H,
                dt,
            )
            new_x = max(CAR_W, min(MAZE_W - CAR_W, new_x))
            new_y = max(CAR_H, min(MAZE_H - CAR_H, new_y))
            car_x, car_y, car_angle = new_x, new_y, new_angle

            trail.append((int(car_x), int(car_y)))
            if len(trail) > MAX_TRAIL:
                trail.pop(0)

            moved = math.hypot(car_x - prev_pos[0], car_y - prev_pos[1])
            if moved < 0.5:
                stuck_timer += dt
            else:
                stuck_timer = 0.0
            prev_pos = (car_x, car_y)

            if stuck_timer > 4.0:
                status_msg = "Car appears stuck — try Reset"
                status_color = RED
            else:
                status_msg = f"Navigating with {ALGO_NAMES[dropdown.selected]}…"
                status_color = DARK_GRAY

        if goal_reached and not reached:
            reached = True
            status_msg = "Goal reached!"
            status_color = GREEN

        # ── Draw ──────────────────────────────────────────────────────────────
        screen.fill(BG_COLOR)

        # Maze area
        maze_surf = pygame.Surface((MAZE_W, MAZE_H))
        maze_surf.fill(MAZE_BG)

        # Grid
        for gx in range(0, MAZE_W, 40):
            pygame.draw.line(maze_surf, (230, 230, 235), (gx, 0), (gx, MAZE_H))
        for gy in range(0, MAZE_H, 40):
            pygame.draw.line(maze_surf, (230, 230, 235), (0, gy), (MAZE_W, gy))

        # Trail
        if len(trail) > 1:
            for i in range(1, len(trail)):
                alpha = int(60 + 140 * i / len(trail))
                c = (50, 120, 220, alpha)
                pygame.draw.line(maze_surf, (50, 120, 220), trail[i - 1], trail[i], 2)

        # Obstacles
        for ox, oy, ow, oh in obstacles:
            pygame.draw.rect(maze_surf, OBS_COLOR, (ox, oy, ow, oh), border_radius=4)
            pygame.draw.rect(maze_surf, OBS_BORDER, (ox, oy, ow, oh), 2, border_radius=4)

        # Goal marker
        pygame.draw.circle(maze_surf, GOAL_COLOR, (GOAL_X, GOAL_Y), 18)
        pygame.draw.circle(maze_surf, WHITE, (GOAL_X, GOAL_Y), 18, 3)
        g_lbl = font_md.render("GOAL", True, WHITE)
        maze_surf.blit(g_lbl, (GOAL_X - g_lbl.get_width() // 2,
                                GOAL_Y - g_lbl.get_height() // 2))

        # Start marker
        pygame.draw.circle(maze_surf, START_COLOR, (START_X, START_Y), 18)
        pygame.draw.circle(maze_surf, WHITE, (START_X, START_Y), 18, 3)
        s_lbl = font_sm.render("START", True, WHITE)
        maze_surf.blit(s_lbl, (START_X - s_lbl.get_width() // 2,
                                START_Y - s_lbl.get_height() // 2))

        # Car
        draw_car(maze_surf, int(car_x), int(car_y), car_angle,
                 color=GREEN if reached else BLUE)

        screen.blit(maze_surf, (0, 0))

        # Sidebar
        pygame.draw.rect(screen, DARK_GRAY, (sidebar_x, 0, SIDEBAR_W, SCREEN_H))
        pygame.draw.line(screen, GRAY, (sidebar_x, 0), (sidebar_x, SCREEN_H), 2)

        title = font_title.render("Self-Driving", True, WHITE)
        title2 = font_title.render("Car Sim", True, WHITE)
        screen.blit(title, (sidebar_x + (SIDEBAR_W - title.get_width()) // 2, 12))
        screen.blit(title2, (sidebar_x + (SIDEBAR_W - title2.get_width()) // 2, 32))

        algo_lbl = font_sm.render("Algorithm:", True, LIGHT_GRAY)
        screen.blit(algo_lbl, (sidebar_x + pad, 62))

        dropdown.draw(screen)
        reset_btn.draw(screen)

        # Info panel
        info_y = 282
        pygame.draw.rect(screen, (60, 65, 80),
                         (sidebar_x + pad, info_y, btn_w, 130), border_radius=6)
        infos = [
            ("Pos X", f"{car_x:.0f}"),
            ("Pos Y", f"{car_y:.0f}"),
            ("Angle", f"{math.degrees(car_angle):.1f}°"),
            ("Dist",  f"{math.hypot(GOAL_X-car_x, GOAL_Y-car_y):.0f}px"),
        ]
        for i, (k, v) in enumerate(infos):
            ky = info_y + 8 + i * 28
            k_txt = font_sm.render(k + ":", True, LIGHT_GRAY)
            v_txt = font_md.render(v, True, WHITE)
            screen.blit(k_txt, (sidebar_x + pad + 6, ky))
            screen.blit(v_txt, (sidebar_x + SIDEBAR_W - v_txt.get_width() - pad - 6, ky))

        # Status
        status_y = SCREEN_H - 80
        pygame.draw.rect(screen, (50, 55, 70),
                         (sidebar_x + pad, status_y, btn_w, 60), border_radius=6)
        words = status_msg.split()
        lines = []
        line = ""
        for w in words:
            test = (line + " " + w).strip()
            if font_sm.size(test)[0] < btn_w - 10:
                line = test
            else:
                lines.append(line)
                line = w
        if line:
            lines.append(line)
        for i, ln in enumerate(lines[:3]):
            lt = font_sm.render(ln, True, status_color)
            screen.blit(lt, (sidebar_x + pad + 5, status_y + 8 + i * 18))

        # Legend
        legend_y = 430
        legends = [
            (BLUE,        "Car"),
            (GOAL_COLOR,  "Goal"),
            (START_COLOR, "Start"),
            (OBS_COLOR,   "Obstacle"),
        ]
        leg_title = font_sm.render("Legend:", True, LIGHT_GRAY)
        screen.blit(leg_title, (sidebar_x + pad, legend_y - 16))
        for i, (c, lbl) in enumerate(legends):
            ly = legend_y + i * 22
            pygame.draw.rect(screen, c, (sidebar_x + pad, ly + 4, 14, 14), border_radius=3)
            lt = font_sm.render(lbl, True, LIGHT_GRAY)
            screen.blit(lt, (sidebar_x + pad + 20, ly + 3))

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
