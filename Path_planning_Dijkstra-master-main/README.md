# Path Planning for Autonomous Vehicles — Dijkstra's Algorithm

**GitHub:** https://github.com/OfirHaf/Autonomic-Vehicle

---

## Introduction

This project implements **Dijkstra's shortest-path algorithm** for an Autonomous Vehicle (AV) navigating a 2-D environment with static obstacles and **terrain cost zones**.

Unlike A*, Dijkstra uses **no heuristic** — it expands nodes strictly by cumulative cost from the start. This guarantees finding the globally optimal path in any weighted graph, at the cost of exploring more of the map.

### AV-Specific Feature — Weighted Terrain

A real autonomous vehicle does not treat all drivable space equally. Roads are cheap to traverse; sidewalks and rough terrain are expensive. Three terrain cost zones are modelled:

| Zone | Cost Multiplier | Represents |
|------|-----------------|-----------|
| Road (dark green bands) | ×1.0 | Paved lanes |
| Open space (default) | ×2.5 | Parking lots, plazas |
| Off-road (dark blue bands) | ×6.0 | Sidewalks, rough terrain |

Dijkstra naturally routes the vehicle through the cheapest path — preferring roads even if they are not the geometrically shortest route.

---

## Algorithm — How Dijkstra Works

1. Assign cost ∞ to every node; set start cost to 0.
2. Push start into a **min-heap** (priority queue).
3. Pop the node with the **lowest cumulative cost**.
4. For each unvisited neighbour, compute `new_cost = current_cost + edge_weight`.
5. If `new_cost < neighbour's current cost`, update it and push to the heap.
6. Repeat until the goal is popped (optimal path guaranteed) or the heap is empty (no path).

**Edge weight formula:**
```
weight = base_move_cost × average_terrain_cost(from, to)
```
- Straight move: `base = 1.0 × step_size`
- Diagonal move: `base = √2 × step_size`

---

## Screenshots

Interactive GUI showing the obstacle map and terrain cost zones:
![Screenshot 1 — Obstacle Map](Screenshot1.png)

Dijkstra exploration (blue cells) and the final optimal path (teal):
![Screenshot 2 — Path Found](Screenshot2.png)

---

## Project Structure

```
Autonomic-Vehicle/
├── Code/
│   ├── utils.py        # Dijkstra class — algorithm, terrain zones, CLI animation
│   ├── gui.py          # Main GUI application (DijkstraGUI)
│   ├── gui_canvas.py   # Canvas rendering — terrain, obstacles, path cells, markers
│   ├── gui_config.py   # Colors, grid settings, terrain zones (derived from utils.py)
│   └── dijkstra.py     # Command-line entry point
├── Screenshot1.png     # GUI — obstacle map view
├── Screenshot2.png     # GUI — exploration + optimal path view
└── README.md
```

### Module Responsibilities

| File | Responsibility |
|------|---------------|
| `utils.py` | `Dijkstra` class: boundary checks, obstacle map, 8-directional search, terrain costs, CLI video export. Also owns `ROAD_ROWS` / `OFFROAD_ROWS` — the single source of truth for zone boundaries. |
| `gui_config.py` | All visual constants: colors, grid size, slider ranges, defaults. Imports zone boundaries from `utils.py` and derives `TERRAIN_ZONES` automatically — no duplication. |
| `gui_canvas.py` | `PathCanvas` (subclass of `tk.Canvas`): draws terrain shading, inflated obstacles, explored cells, final path, and start/goal markers. |
| `gui.py` | `DijkstraGUI`: wires sliders, buttons, canvas, and animation loop. Entry point via `main()`. |
| `dijkstra.py` | CLI wrapper: prompts for inputs, runs search, exports `dijkstra_path.avi`. Safe to import — all logic is under `if __name__ == "__main__":`. |

---

## Requirements

- Python 3.8+
- numpy
- OpenCV (`cv2`) — CLI video export only
- tkinter — included with Python on Windows/macOS

```bash
pip install numpy opencv-python
```

---

## How to Run

### 1 — Clone the repository

```bash
git clone https://github.com/OfirHaf/Autonomic-Vehicle.git
cd Autonomic-Vehicle
```

### 2 — Install dependencies

```bash
pip install numpy opencv-python
```

> `tkinter` is included with Python on Windows and macOS. On Ubuntu/Debian run `sudo apt install python3-tk`.

### 3 — Launch the GUI (Recommended)

```bash
cd Code
python gui.py
```

The window opens with the obstacle map and terrain zones pre-rendered. No further setup required.

#### Controls

| Action | Input |
|--------|-------|
| Set **start** point | Left-click on the canvas |
| Set **goal** point | Right-click on the canvas |
| Run Dijkstra | Click **RUN** or press `Enter` |
| Stop animation | Click **STOP** or press `Escape` |
| Reset everything | Click **RESET** or press `R` |

#### Parameter Sliders

| Slider | Effect |
|--------|--------|
| Robot Radius | Inflates all obstacle boundaries to fit the robot body |
| Clearance | Extra safety margin on top of the robot radius |
| Step Size | Grid movement granularity (1 = finest path, 10 = coarser but faster) |
| Anim Speed | Controls animation frame rate — higher = faster playback |

**Tip:** Place start and goal in different terrain zones and watch how the path bends toward the green road bands to minimise weighted cost.

### 4 — Command-Line Mode (exports AVI video)

```bash
cd Code
python dijkstra.py
```

Prompts for start/goal coordinates, robot radius, clearance, and step size.  
Outputs: optimal weighted distance, nodes explored, path length, and `dijkstra_path.avi`.

---

## Obstacle Map

The map is a 300 × 200 grid containing:

| Obstacle | Shape |
|----------|-------|
| Circle | Centre (150, 225), radius 25 |
| Ellipse | Centre (100, 150), axes 20 × 40 |
| Triangle 1 | Vertices approx. (120, 20), (150, 50), (185, 25) |
| Triangle 2 | Vertices approx. (150, 50), (185, 25), (185, 75) |
| Rhombus | Centre (25, 225) |
| Rectangle | Diamond-square shape near (150, 75) |
| Rod | Diagonal bar near (60, 65) |

Robot radius and clearance inflate each obstacle boundary during path validation.

---

## Key Terms

Dijkstra's Algorithm · Graph-Based Navigation · Weighted Path Planning · Autonomous Vehicle · State-Space Exploration · Terrain Cost Map · Priority Queue · Min-Heap

---

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| Python | 3.8+ | Runtime |
| numpy | any | Image array for CLI video |
| opencv-python | any | CLI video export |
| tkinter | built-in | GUI framework |
