# Configuration and color scheme for the Dijkstra Path Planning GUI.
#
# Design principle: gui_config.py is a SINGLE SOURCE OF TRUTH for all
# visual constants. It never duplicates algorithmic data — instead it
# IMPORTS zone boundaries from utils.py, so the canvas shading and the
# Dijkstra cost model are guaranteed to stay in sync.

from utils import ROAD_ROWS, OFFROAD_ROWS   # terrain zone boundaries (never duplicate these here)

# ---------------------------------------------------------------------------
# Grid dimensions — must match the Dijkstra class environment in utils.py
# ---------------------------------------------------------------------------
GRID_WIDTH  = 300    # number of columns (x-axis)
GRID_HEIGHT = 200    # number of rows    (y-axis, row 1 = bottom)

# Scale factor: each logical grid cell is rendered as SCALE×SCALE pixels.
# Increase for a larger window; decrease for a smaller one.
SCALE = 2

# Derived canvas pixel dimensions (do not edit directly — change SCALE above)
CANVAS_WIDTH  = GRID_WIDTH  * SCALE   # 600 pixels at default scale
CANVAS_HEIGHT = GRID_HEIGHT * SCALE   # 400 pixels at default scale

# ---------------------------------------------------------------------------
# Color scheme — blue/teal dark theme
# All colors are hex strings consumed by tkinter.
# ---------------------------------------------------------------------------
COLORS = {
    # ---------- Application chrome ----------
    'background':       '#0d1b2a',   # main window background (deep navy)
    'canvas_bg':        '#0a1628',   # canvas default cell color (slightly darker navy)

    # ---------- Obstacles ----------
    'obstacle':         '#e63946',   # inflated obstacle fill (vivid red)
    'obstacle_outline': '#ff8fa3',   # obstacle border (light pink, unused currently)

    # ---------- Terrain zones (background shading on canvas) ----------
    'terrain_road':     '#1a3a1a',   # dark green  — cost ×1.0 paved road bands
    'terrain_normal':   '#0a1628',   # default bg  — cost ×2.5 open space (matches canvas_bg)
    'terrain_offroad':  '#1a1a3a',   # dark blue   — cost ×6.0 rough terrain / sidewalk

    # ---------- Start / goal markers ----------
    'start':            '#06d6a0',   # teal green  — start point oval
    'start_outline':    '#ffffff',   # white border around start marker
    'goal':             '#ffd166',   # amber       — goal point oval
    'goal_outline':     '#ffffff',   # white border around goal marker

    # ---------- Search visualization ----------
    'explored':         '#457b9d',   # steel blue  — cells expanded by Dijkstra
    'path':             '#06d6a0',   # teal        — cells on the final optimal path
    'path_outline':     '#ffffff',   # thin white outline on path cells

    # ---------- UI text and controls ----------
    'text':             '#e0fbfc',   # primary text (near-white cyan)
    'text_secondary':   '#6c757d',   # hint / label text (grey)
    'button_bg':        '#1d3557',   # default button background (dark blue)
    'button_fg':        '#e0fbfc',   # button label color
    'button_active':    '#2d5a8e',   # button hover/active highlight
    'slider_trough':    '#1d3557',   # slider track background
}

# ---------------------------------------------------------------------------
# Default slider values — shown on first launch
# ---------------------------------------------------------------------------
DEFAULTS = {
    'radius':           0,    # robot body radius (0 = point robot)
    'clearance':        0,    # safety margin around obstacles (0 = no extra margin)
    'step_size':        1,    # movement granularity in grid cells (1 = finest)
    'animation_speed':  50,   # higher value = faster animation (max speed by default)
}

# ---------------------------------------------------------------------------
# Slider min/max ranges
# Each tuple is (min, max) passed directly to ttk.Scale(from_=, to=).
# ---------------------------------------------------------------------------
PARAM_RANGES = {
    'radius':          (0, 20),    # 0 = point robot, 20 = large vehicle footprint
    'clearance':       (0, 20),    # 0 = no clearance, 20 = very cautious
    'step_size':       (1, 10),    # 1 = finest path, 10 = coarse (faster search)
    'animation_speed': (1, 50),    # 1 = slowest (50 ms/frame), 50 = fastest (1 ms/frame)
}

# Start/goal marker radius in pixels (drawn as ovals on the canvas)
POINT_MARKER_SIZE = 8

# ---------------------------------------------------------------------------
# Terrain zone definitions — derived from utils.py to stay in sync.
#
# Format: list of (row_lo, row_hi, color_key) tuples.
# gui_canvas.py iterates this list to shade terrain bands on the canvas.
# ---------------------------------------------------------------------------
TERRAIN_ZONES = (
    [(lo, hi, 'terrain_road')    for lo, hi in ROAD_ROWS] +    # green road bands
    [(lo, hi, 'terrain_offroad') for lo, hi in OFFROAD_ROWS]   # blue off-road bands
)
