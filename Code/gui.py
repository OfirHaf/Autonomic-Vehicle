#!/usr/bin/env python3
"""
Dijkstra Path Planning Visualizer — Interactive GUI

Features:
- Terrain cost visualization (road / normal / off-road zones)
- Click to set start (left-click) and goal (right-click)
- Adjustable parameters: robot radius, clearance, step size
- Animated Dijkstra search with speed control
- Blue/teal dark theme
"""

import tkinter as tk
from tkinter import ttk
from utils import Dijkstra
from gui_config import (
    GRID_WIDTH, GRID_HEIGHT, SCALE, COLORS, DEFAULTS, PARAM_RANGES
)
from gui_canvas import PathCanvas


class DijkstraGUI:
    """Main GUI application for Dijkstra path planning visualization."""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Dijkstra Path Planning Visualizer — AV Project")
        self.root.configure(bg=COLORS['background'])
        self.root.resizable(False, False)

        # Animation state
        self.is_animating  = False
        self.animation_id  = None
        self.explored_states = []
        self.path_states   = []
        self.anim_index    = 0

        self._setup_styles()
        self._setup_ui()
        self._bind_events()

        # Initial render: terrain → obstacles
        self.root.after(100, self._full_redraw)

    # ------------------------------------------------------------------
    # UI setup
    # ------------------------------------------------------------------
    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TScale',
            background=COLORS['background'],
            troughcolor=COLORS['slider_trough'],
        )
        style.configure('TButton',
            background=COLORS['button_bg'],
            foreground=COLORS['button_fg'],
            padding=(20, 10),
            font=('Helvetica', 11, 'bold')
        )
        style.map('TButton',
            background=[('active', COLORS['button_active'])]
        )

    def _setup_ui(self):
        main = tk.Frame(self.root, bg=COLORS['background'])
        main.pack(padx=10, pady=8)

        # Title
        tk.Label(
            main,
            text="Dijkstra Path Planning Visualizer",
            font=('Helvetica', 14, 'bold'),
            fg=COLORS['text'],
            bg=COLORS['background']
        ).pack(pady=(0, 2))

        tk.Label(
            main,
            text="Left-click: Set Start  |  Right-click: Set Goal  |  Terrain: Road(×1.0)  Open(×2.5)  Off-road(×6.0)",
            font=('Helvetica', 8),
            fg=COLORS['text_secondary'],
            bg=COLORS['background']
        ).pack(pady=(0, 4))

        # Canvas
        self.canvas = PathCanvas(main, Dijkstra)
        self.canvas.pack(pady=4)

        self._setup_controls(main)

    def _setup_controls(self, parent):
        ctrl = tk.Frame(parent, bg=COLORS['background'])
        ctrl.pack(fill='x', pady=4)

        # Left — coordinate display
        coord_frame = tk.Frame(ctrl, bg=COLORS['background'])
        coord_frame.pack(side='left', padx=20)

        self.start_label = tk.Label(
            coord_frame, text="Start: Not set",
            font=('Helvetica', 11), fg=COLORS['start'],
            bg=COLORS['background'], width=20, anchor='w'
        )
        self.start_label.pack(anchor='w')

        self.goal_label = tk.Label(
            coord_frame, text="Goal: Not set",
            font=('Helvetica', 11), fg=COLORS['goal'],
            bg=COLORS['background'], width=20, anchor='w'
        )
        self.goal_label.pack(anchor='w')

        # Middle — parameter sliders
        params = tk.Frame(ctrl, bg=COLORS['background'])
        params.pack(side='left', padx=40)

        self._create_slider(params, "Robot Radius:", 'radius',
            *PARAM_RANGES['radius'], DEFAULTS['radius'])
        self._create_slider(params, "Clearance:", 'clearance',
            *PARAM_RANGES['clearance'], DEFAULTS['clearance'])
        self._create_slider(params, "Step Size:", 'step_size',
            *PARAM_RANGES['step_size'], DEFAULTS['step_size'])

        # Right — animation speed
        speed_frame = tk.Frame(ctrl, bg=COLORS['background'])
        speed_frame.pack(side='left', padx=40)

        self._create_slider(speed_frame, "Anim Speed:", 'speed',
            *PARAM_RANGES['animation_speed'], DEFAULTS['animation_speed'])


        # Buttons
        btn_frame = tk.Frame(parent, bg=COLORS['background'])
        btn_frame.pack(pady=6)

        self.run_button = tk.Button(
            btn_frame, text="RUN",
            command=self._on_run,
            font=('Helvetica', 12, 'bold'),
            bg=COLORS['start'], fg='#000000',
            activebackground='#04b885',
            width=10, height=1, cursor='hand2'
        )
        self.run_button.pack(side='left', padx=10)

        self.stop_button = tk.Button(
            btn_frame, text="STOP",
            command=self._on_stop,
            font=('Helvetica', 12, 'bold'),
            bg=COLORS['obstacle'], fg='#ffffff',
            activebackground='#cc2233',
            width=10, height=1, cursor='hand2',
            state='disabled'
        )
        self.stop_button.pack(side='left', padx=10)

        self.reset_button = tk.Button(
            btn_frame, text="RESET",
            command=self._on_reset,
            font=('Helvetica', 12, 'bold'),
            bg=COLORS['button_bg'], fg=COLORS['button_fg'],
            activebackground=COLORS['button_active'],
            width=10, height=1, cursor='hand2'
        )
        self.reset_button.pack(side='left', padx=10)

        # Status bar
        status_frame = tk.Frame(parent, bg=COLORS['background'])
        status_frame.pack(pady=2)

        self.status_label = tk.Label(
            status_frame, text="Status: Ready",
            font=('Helvetica', 11), fg=COLORS['text'],
            bg=COLORS['background']
        )
        self.status_label.pack(side='left', padx=20)

        self.distance_label = tk.Label(
            status_frame, text="Weighted Distance: ---",
            font=('Helvetica', 11), fg=COLORS['path'],
            bg=COLORS['background']
        )
        self.distance_label.pack(side='left', padx=20)

        self.explored_label = tk.Label(
            status_frame, text="Nodes Explored: ---",
            font=('Helvetica', 11), fg=COLORS['explored'],
            bg=COLORS['background']
        )
        self.explored_label.pack(side='left', padx=20)

    def _create_slider(self, parent, label_text, var_name,
                       min_val, max_val, default):
        frame = tk.Frame(parent, bg=COLORS['background'])
        frame.pack(fill='x', pady=2)

        tk.Label(frame, text=label_text,
            font=('Helvetica', 10), fg=COLORS['text'],
            bg=COLORS['background'], width=15, anchor='e'
        ).pack(side='left')

        var = tk.IntVar(value=default)
        setattr(self, f'{var_name}_var', var)

        ttk.Scale(
            frame,
            from_=min_val, to=max_val,
            variable=var, orient='horizontal', length=120,
            command=lambda v, n=var_name: self._on_slider_change(n)
        ).pack(side='left', padx=5)

        tk.Label(frame, textvariable=var,
            font=('Helvetica', 10, 'bold'), fg=COLORS['text'],
            bg=COLORS['background'], width=3
        ).pack(side='left')

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------
    def _bind_events(self):
        self.canvas.bind("<Button-1>", self._on_left_click)
        self.canvas.bind("<Button-2>", self._on_right_click)
        self.canvas.bind("<Button-3>", self._on_right_click)

    def _on_slider_change(self, var_name):
        if var_name in ('radius', 'clearance'):
            self._full_redraw()

    def _full_redraw(self):
        """Redraw terrain then obstacles (order matters for z-layering)."""
        if self.is_animating:
            return
        self.canvas.draw_terrain()
        self.canvas.draw_obstacles(
            self.radius_var.get(), self.clearance_var.get()
        )
        self.canvas.raise_markers()

    def _on_left_click(self, event):
        if self.is_animating:
            return
        row, col = self.canvas.canvas_to_grid(event.x, event.y)
        if not self._is_valid(row, col):
            self.status_label.config(
                text="Status: Invalid position!", fg=COLORS['obstacle'])
            return
        self.canvas.set_start(row, col)
        self.start_label.config(text=f"Start: ({col}, {row})")
        self.status_label.config(text="Status: Start set", fg=COLORS['text'])
        self.canvas.clear_path()

    def _on_right_click(self, event):
        if self.is_animating:
            return
        row, col = self.canvas.canvas_to_grid(event.x, event.y)
        if not self._is_valid(row, col):
            self.status_label.config(
                text="Status: Invalid position!", fg=COLORS['obstacle'])
            return
        self.canvas.set_goal(row, col)
        self.goal_label.config(text=f"Goal: ({col}, {row})")
        self.status_label.config(text="Status: Goal set", fg=COLORS['text'])
        self.canvas.clear_path()

    def _is_valid(self, row, col):
        r, c = self.radius_var.get(), self.clearance_var.get()
        tmp = Dijkstra((row, col), (row, col), c, r, 1)
        return tmp.IsValid(row, col) and not tmp.IsObstacle(row, col)

    # ------------------------------------------------------------------
    # Run / Stop / Reset
    # ------------------------------------------------------------------
    def _on_run(self):
        if self.is_animating:
            return
        if not self.canvas.start_point:
            self.status_label.config(
                text="Status: Set start point first!", fg=COLORS['obstacle'])
            return
        if not self.canvas.goal_point:
            self.status_label.config(
                text="Status: Set goal point first!", fg=COLORS['obstacle'])
            return

        self.canvas.clear_path()

        start     = self.canvas.start_point
        goal      = self.canvas.goal_point
        radius    = self.radius_var.get()
        clearance = self.clearance_var.get()
        step_size = self.step_size_var.get()

        self.status_label.config(
            text="Status: Running Dijkstra...", fg=COLORS['explored'])
        self.root.update()

        dijkstra = Dijkstra(start, goal, clearance, radius, step_size)

        if not dijkstra.IsValid(start[0], start[1]) or \
                dijkstra.IsObstacle(start[0], start[1]):
            self.status_label.config(
                text="Status: Start is invalid with current params!",
                fg=COLORS['obstacle'])
            return
        if not dijkstra.IsValid(goal[0], goal[1]) or \
                dijkstra.IsObstacle(goal[0], goal[1]):
            self.status_label.config(
                text="Status: Goal is invalid with current params!",
                fg=COLORS['obstacle'])
            return

        explored, path, distance = dijkstra.search()

        self.explored_states = explored
        self.path_states     = path
        self.anim_index      = 0

        self.explored_label.config(
            text=f"Nodes Explored: {len(explored)}")

        if distance == float('inf') or not path:
            self.status_label.config(
                text="Status: No path found — try lower Radius/Clearance or different points",
                fg=COLORS['obstacle'])
            self.distance_label.config(text="Weighted Distance: ---")
        else:
            self.distance_label.config(
                text=f"Weighted Distance: {distance:.2f}")

        self.is_animating = True
        self.run_button.config(state='disabled')
        self.stop_button.config(state='normal')
        self._animate_step()

    def _animate_step(self):
        if not self.is_animating:
            return

        batch = max(1, len(self.explored_states) // 200)
        for _ in range(batch):
            if self.anim_index < len(self.explored_states):
                self.canvas.draw_explored_cell(
                    self.explored_states[self.anim_index])
                self.anim_index += 1

        pct = int(self.anim_index / len(self.explored_states) * 100) \
              if self.explored_states else 100
        self.status_label.config(
            text=f"Status: Exploring... {pct}%", fg=COLORS['explored'])
        self.canvas.update_idletasks()

        if self.anim_index < len(self.explored_states):
            delay = self.speed_var.get()
            self.animation_id = self.root.after(delay, self._animate_step)
        else:
            self._draw_final_path()

    def _draw_final_path(self):
        self.canvas.raise_markers()
        if self.path_states:
            for state in self.path_states:
                self.canvas.draw_path_cell(state)
            self.canvas.raise_markers()
            self.status_label.config(
                text="Status: Path found!", fg=COLORS['path'])
        else:
            self.status_label.config(
                text="Status: No path found — try lower Radius/Clearance or different points",
                fg=COLORS['obstacle'])
        self.is_animating = False
        self.run_button.config(state='normal')
        self.stop_button.config(state='disabled')

    def _on_stop(self):
        if self.animation_id:
            self.root.after_cancel(self.animation_id)
            self.animation_id = None
        self.is_animating = False
        self.run_button.config(state='normal')
        self.stop_button.config(state='disabled')
        self.status_label.config(text="Status: Stopped", fg=COLORS['text'])

    def _on_reset(self):
        self._on_stop()
        self.canvas.clear_all()
        self.start_label.config(text="Start: Not set")
        self.goal_label.config(text="Goal: Not set")
        self.status_label.config(text="Status: Ready", fg=COLORS['text'])
        self.distance_label.config(text="Weighted Distance: ---")
        self.explored_label.config(text="Nodes Explored: ---")
        self.explored_states = []
        self.path_states = []
        self._full_redraw()

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------
    def run(self):
        self.root.mainloop()


def main():
    app = DijkstraGUI()
    app.run()


if __name__ == "__main__":
    main()
