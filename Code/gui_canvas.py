# Canvas rendering for the Dijkstra Path Planning GUI.
#
# PathCanvas is a subclass of tk.Canvas that owns all drawing operations:
#   - Terrain zone shading (road / normal / off-road bands)
#   - Obstacle cells (rendered by querying Dijkstra.IsObstacle)
#   - Explored cells (animation — shown as the algorithm expands nodes)
#   - Path cells (final optimal path — drawn after exploration completes)
#   - Start / goal markers (oval overlays, always raised to the top layer)
#
# Coordinate system:
#   Grid  — (row, col): row 1 = bottom, row GRID_HEIGHT = top
#   Canvas — (x, y):   (0, 0) = top-left corner (standard tkinter)
#   Conversion is done by grid_to_canvas / canvas_to_grid helpers.

import tkinter as tk
from gui_config import (
    GRID_WIDTH, GRID_HEIGHT, SCALE, CANVAS_WIDTH, CANVAS_HEIGHT,
    COLORS, POINT_MARKER_SIZE, TERRAIN_ZONES
)


class PathCanvas(tk.Canvas):
    """Custom canvas that renders terrain zones, obstacles, and the Dijkstra path."""

    def __init__(self, parent, dijkstra_class, **kwargs):
        super().__init__(
            parent,
            width=CANVAS_WIDTH,
            height=CANVAS_HEIGHT,
            bg=COLORS['canvas_bg'],
            highlightthickness=2,
            highlightbackground=COLORS['text_secondary'],
            **kwargs
        )
        self.dijkstra_class  = dijkstra_class  # Dijkstra class reference (used to query IsObstacle)
        self.start_point     = None            # (row, col) or None if not set
        self.goal_point      = None            # (row, col) or None if not set

        # Track canvas item IDs so we can delete them efficiently by category
        self.terrain_items   = []   # background terrain rectangles
        self.obstacle_items  = []   # inflated obstacle rectangles
        self.explored_items  = []   # exploration animation cells
        self.path_items      = []   # final path cells
        self.start_marker    = None # oval item ID for start point
        self.goal_marker     = None # oval item ID for goal point

    # ------------------------------------------------------------------
    # Coordinate conversion
    #
    # Grid uses (row, col) where row 1 is at the BOTTOM of the canvas.
    # Canvas uses (x, y) where y=0 is at the TOP.
    # ------------------------------------------------------------------
    def grid_to_canvas(self, row, col):
        """Convert grid (row, col) → canvas top-left (x, y) of the cell."""
        x = (col - 1) * SCALE          # col 1 → x=0, col 2 → x=SCALE, …
        y = (GRID_HEIGHT - row) * SCALE # row 1 → y=CANVAS_HEIGHT-SCALE (bottom)
        return x, y

    def canvas_to_grid(self, x, y):
        """Convert canvas pixel (x, y) → grid (row, col)."""
        col = (x // SCALE) + 1
        row = GRID_HEIGHT - (y // SCALE)
        return row, col

    # ------------------------------------------------------------------
    # Terrain zone rendering
    #
    # Called once on startup and again whenever radius/clearance sliders
    # change (since obstacle inflation can visually shift zone boundaries).
    # Draws a coloured rectangle for each (row_lo, row_hi) band.
    # ------------------------------------------------------------------
    def draw_terrain(self):
        """Draw terrain zone bands as background shading."""
        # Remove old terrain rectangles before redrawing
        for item in self.terrain_items:
            self.delete(item)
        self.terrain_items = []

        for row_lo, row_hi, color_key in TERRAIN_ZONES:
            color = COLORS[color_key]
            # Convert row band → canvas y-coordinates
            # row_hi is nearer the top (smaller y), row_lo is nearer the bottom (larger y)
            x0 = 0
            x1 = CANVAS_WIDTH
            y0 = (GRID_HEIGHT - row_hi) * SCALE       # top edge of the band
            y1 = (GRID_HEIGHT - row_lo + 1) * SCALE   # bottom edge (+1 includes row_lo)
            item = self.create_rectangle(
                x0, y0, x1, y1,
                fill=color,
                outline='',        # no border — bands are seamless
                tags='terrain'
            )
            self.terrain_items.append(item)

    # ------------------------------------------------------------------
    # Obstacle rendering
    #
    # Iterates every cell and calls Dijkstra.IsObstacle — analytic shapes
    # mean no separate obstacle list is needed. The result respects the
    # current radius and clearance, so obstacles visually grow when sliders
    # are increased.
    # ------------------------------------------------------------------
    def draw_obstacles(self, radius=0, clearance=0):
        """Draw all obstacle cells using Dijkstra.IsObstacle()."""
        # Remove previously drawn obstacles (e.g. when slider values changed)
        for item in self.obstacle_items:
            self.delete(item)
        self.obstacle_items = []

        # Temporary Dijkstra instance just for obstacle queries — start/goal
        # don't matter here; we only use IsObstacle.
        temp = self.dijkstra_class((1, 1), (1, 1), clearance, radius, 1)

        for row in range(1, GRID_HEIGHT + 1):
            for col in range(1, GRID_WIDTH + 1):
                if temp.IsObstacle(row, col):
                    x, y = self.grid_to_canvas(row, col)
                    item = self.create_rectangle(
                        x, y, x + SCALE, y + SCALE,
                        fill=COLORS['obstacle'],
                        outline='',           # no border for performance (60k cells)
                        tags='obstacle'
                    )
                    self.obstacle_items.append(item)

    # ------------------------------------------------------------------
    # Cell drawing helpers
    # ------------------------------------------------------------------
    def draw_cell(self, row, col, color, tag='cell'):
        """Draw a single SCALE×SCALE cell at (row, col) with the given color."""
        x, y = self.grid_to_canvas(row, col)
        return self.create_rectangle(
            x, y, x + SCALE, y + SCALE,
            fill=color, outline='', tags=tag
        )

    def draw_explored_cell(self, state):
        """Paint a cell as 'explored' (expanded by Dijkstra). Called per animation frame."""
        row, col = state
        item = self.draw_cell(row, col, COLORS['explored'], 'explored')
        self.explored_items.append(item)
        return item

    def draw_path_cell(self, state):
        """Paint a cell as part of the final optimal path. Drawn after exploration ends."""
        row, col = state
        x, y = self.grid_to_canvas(row, col)
        item = self.create_rectangle(
            x, y, x + SCALE, y + SCALE,
            fill=COLORS['path'],
            outline=COLORS['path_outline'],
            width=1,              # thin white outline makes the path pop visually
            tags='path'
        )
        self.path_items.append(item)
        return item

    # ------------------------------------------------------------------
    # Start / goal markers — oval overlays drawn on top of everything else
    # ------------------------------------------------------------------
    def set_start(self, row, col):
        """Place or move the start marker oval to (row, col)."""
        self.start_point = (row, col)
        if self.start_marker:
            self.delete(self.start_marker)   # remove previous marker
        x, y = self.grid_to_canvas(row, col)
        cx, cy = x + SCALE // 2, y + SCALE // 2   # centre of the cell
        self.start_marker = self.create_oval(
            cx - POINT_MARKER_SIZE, cy - POINT_MARKER_SIZE,
            cx + POINT_MARKER_SIZE, cy + POINT_MARKER_SIZE,
            fill=COLORS['start'], outline=COLORS['start_outline'],
            width=2, tags='start'
        )
        self.tag_raise('start')    # always on top of terrain and obstacles

    def set_goal(self, row, col):
        """Place or move the goal marker oval to (row, col)."""
        self.goal_point = (row, col)
        if self.goal_marker:
            self.delete(self.goal_marker)    # remove previous marker
        x, y = self.grid_to_canvas(row, col)
        cx, cy = x + SCALE // 2, y + SCALE // 2
        self.goal_marker = self.create_oval(
            cx - POINT_MARKER_SIZE, cy - POINT_MARKER_SIZE,
            cx + POINT_MARKER_SIZE, cy + POINT_MARKER_SIZE,
            fill=COLORS['goal'], outline=COLORS['goal_outline'],
            width=2, tags='goal'
        )
        self.tag_raise('goal')

    # ------------------------------------------------------------------
    # Clear helpers
    # ------------------------------------------------------------------
    def clear_path(self):
        """Remove explored and path cells, then re-raise start/goal markers."""
        for item in self.explored_items:
            self.delete(item)
        self.explored_items = []
        for item in self.path_items:
            self.delete(item)
        self.path_items = []
        self.raise_markers()   # markers may have been covered during animation

    def clear_all(self):
        """Remove path cells AND start/goal markers (full reset)."""
        self.clear_path()
        if self.start_marker:
            self.delete(self.start_marker)
            self.start_marker = None
        self.start_point = None
        if self.goal_marker:
            self.delete(self.goal_marker)
            self.goal_marker = None
        self.goal_point = None

    def raise_markers(self):
        """Bring start and goal markers to the front of the z-order."""
        self.tag_raise('start')
        self.tag_raise('goal')
