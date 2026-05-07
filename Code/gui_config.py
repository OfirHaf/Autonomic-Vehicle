# Configuration and color scheme for Dijkstra Path Planning GUI
# Mirrors the structure of gui_config.py from the A* example project.

# Grid dimensions (matches Dijkstra class — same environment as A* for comparison)
GRID_WIDTH  = 300
GRID_HEIGHT = 200

# Scale factor: each grid cell is rendered as SCALE×SCALE pixels
SCALE = 2

# Canvas pixel dimensions
CANVAS_WIDTH  = GRID_WIDTH  * SCALE
CANVAS_HEIGHT = GRID_HEIGHT * SCALE

# ---------------------------------------------------------------------------
# Color scheme — blue/teal theme to visually distinguish from the A* project
# ---------------------------------------------------------------------------
COLORS = {
    # Application background
    'background':      '#0d1b2a',
    'canvas_bg':       '#0a1628',

    # Obstacles
    'obstacle':        '#e63946',
    'obstacle_outline':'#ff8fa3',

    # Terrain zones (AV feature — shown as background shading on the canvas)
    'terrain_road':    '#1a3a1a',   # dark green  — paved road band
    'terrain_normal':  '#0a1628',   # default bg  — open space
    'terrain_offroad': '#1a1a3a',   # dark blue   — rough terrain / sidewalk

    # Start / goal markers
    'start':           '#06d6a0',   # teal green
    'start_outline':   '#ffffff',
    'goal':            '#ffd166',   # amber
    'goal_outline':    '#ffffff',

    # Search visualization
    'explored':        '#457b9d',   # steel blue  — Dijkstra expanded nodes
    'path':            '#06d6a0',   # teal        — final optimal path
    'path_outline':    '#ffffff',

    # UI text and controls
    'text':            '#e0fbfc',
    'text_secondary':  '#6c757d',
    'button_bg':       '#1d3557',
    'button_fg':       '#e0fbfc',
    'button_active':   '#2d5a8e',
    'slider_trough':   '#1d3557',
}

# Default slider values
DEFAULTS = {
    'radius':           0,
    'clearance':        0,
    'step_size':        1,
    'animation_speed':  1,   # ms delay per frame (lower = faster)
}

# Slider min/max ranges
PARAM_RANGES = {
    'radius':          (0, 20),
    'clearance':       (0, 20),
    'step_size':       (1, 10),
    'animation_speed': (1, 50),
}

# Start/goal marker radius in pixels
POINT_MARKER_SIZE = 8

# ---------------------------------------------------------------------------
# Terrain zone definitions — must match utils.py ROAD_ROWS / OFFROAD_ROWS
# Used by the canvas to draw background shading before drawing obstacles.
# ---------------------------------------------------------------------------
TERRAIN_ZONES = [
    # (row_lo, row_hi, color_key)
    (30,  50,  'terrain_road'),
    (90,  110, 'terrain_road'),
    (150, 170, 'terrain_road'),
    (1,   20,  'terrain_offroad'),
    (60,  80,  'terrain_offroad'),
    (120, 140, 'terrain_offroad'),
    (180, 200, 'terrain_offroad'),
]
