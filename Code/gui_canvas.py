# Canvas rendering for the Dijkstra Path Planning GUI.
# Extends PathCanvas from the A* example with terrain zone visualization.

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
        self.dijkstra_class  = dijkstra_class
        self.start_point     = None
        self.goal_point      = None
        self.terrain_items   = []
        self.obstacle_items  = []
        self.explored_items  = []
        self.path_items      = []
        self.start_marker    = None
        self.goal_marker     = None

    # ------------------------------------------------------------------
    # Coordinate conversion — identical to the A* example
    # ------------------------------------------------------------------
    def grid_to_canvas(self, row, col):
        """Grid (row, col) → canvas (x, y). Row 1 is at the bottom."""
        x = (col - 1) * SCALE
        y = (GRID_HEIGHT - row) * SCALE
        return x, y

    def canvas_to_grid(self, x, y):
        """Canvas (x, y) → grid (row, col)."""
        col = (x // SCALE) + 1
        row = GRID_HEIGHT - (y // SCALE)
        return row, col

    # ------------------------------------------------------------------
    # Terrain zone rendering (AV-specific: shows cost landscape)
    # ------------------------------------------------------------------
    def draw_terrain(self):
        """Draw terrain zone bands as background shading."""
        for item in self.terrain_items:
            self.delete(item)
        self.terrain_items = []

        for row_lo, row_hi, color_key in TERRAIN_ZONES:
            color = COLORS[color_key]
            # Convert row band to canvas y-coordinates
            # row_hi is at smaller y (higher on canvas), row_lo is larger y
            x0 = 0
            x1 = CANVAS_WIDTH
            y0 = (GRID_HEIGHT - row_hi) * SCALE
            y1 = (GRID_HEIGHT - row_lo + 1) * SCALE
            item = self.create_rectangle(
                x0, y0, x1, y1,
                fill=color,
                outline='',
                tags='terrain'
            )
            self.terrain_items.append(item)

    # ------------------------------------------------------------------
    # Obstacle rendering
    # ------------------------------------------------------------------
    def draw_obstacles(self, radius=0, clearance=0):
        """Draw all obstacle cells using Dijkstra.IsObstacle()."""
        for item in self.obstacle_items:
            self.delete(item)
        self.obstacle_items = []

        temp = self.dijkstra_class((1, 1), (1, 1), clearance, radius, 1)

        for row in range(1, GRID_HEIGHT + 1):
            for col in range(1, GRID_WIDTH + 1):
                if temp.IsObstacle(row, col):
                    x, y = self.grid_to_canvas(row, col)
                    item = self.create_rectangle(
                        x, y, x + SCALE, y + SCALE,
                        fill=COLORS['obstacle'],
                        outline='',
                        tags='obstacle'
                    )
                    self.obstacle_items.append(item)

    # ------------------------------------------------------------------
    # Cell drawing helpers
    # ------------------------------------------------------------------
    def draw_cell(self, row, col, color, tag='cell'):
        x, y = self.grid_to_canvas(row, col)
        return self.create_rectangle(
            x, y, x + SCALE, y + SCALE,
            fill=color, outline='', tags=tag
        )

    def draw_explored_cell(self, state):
        row, col = state
        item = self.draw_cell(row, col, COLORS['explored'], 'explored')
        self.explored_items.append(item)
        return item

    def draw_path_cell(self, state):
        row, col = state
        x, y = self.grid_to_canvas(row, col)
        item = self.create_rectangle(
            x, y, x + SCALE, y + SCALE,
            fill=COLORS['path'],
            outline=COLORS['path_outline'],
            width=1,
            tags='path'
        )
        self.path_items.append(item)
        return item

    # ------------------------------------------------------------------
    # Start / goal markers
    # ------------------------------------------------------------------
    def set_start(self, row, col):
        self.start_point = (row, col)
        if self.start_marker:
            self.delete(self.start_marker)
        x, y = self.grid_to_canvas(row, col)
        cx, cy = x + SCALE // 2, y + SCALE // 2
        self.start_marker = self.create_oval(
            cx - POINT_MARKER_SIZE, cy - POINT_MARKER_SIZE,
            cx + POINT_MARKER_SIZE, cy + POINT_MARKER_SIZE,
            fill=COLORS['start'], outline=COLORS['start_outline'],
            width=2, tags='start'
        )
        self.tag_raise('start')

    def set_goal(self, row, col):
        self.goal_point = (row, col)
        if self.goal_marker:
            self.delete(self.goal_marker)
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
        for item in self.explored_items:
            self.delete(item)
        self.explored_items = []
        for item in self.path_items:
            self.delete(item)
        self.path_items = []
        self.raise_markers()

    def clear_all(self):
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
        self.tag_raise('start')
        self.tag_raise('goal')
