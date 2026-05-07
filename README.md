# Path Planning for Autonomous Vehicles — Dijkstra's Algorithm

---

## Introduction

This project implements **Dijkstra's shortest-path algorithm** for an Autonomous Vehicle (AV) navigating a 2-D environment with static obstacles and **terrain cost zones**.

Unlike A*, Dijkstra uses **no heuristic** — it expands nodes strictly by cumulative cost from the start. This guarantees finding the globally optimal path in any weighted graph, at the cost of exploring more of the map. The project is designed to run alongside the companion A* project so that both algorithms can be visually compared on the same obstacle map.

### AV-Specific Feature — Weighted Terrain

A real autonomous vehicle does not treat all drivable space equally. Roads are cheap to traverse; sidewalks, grassy areas, and rough terrain are expensive or forbidden. This project models three terrain cost zones:

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

## Project Structure

```
Autonomic-Vehicle/
├── Code/
│   ├── dijkstra.py     # Command-line entry point
│   ├── utils.py        # Dijkstra class — all algorithm and terrain logic
│   ├── gui.py          # Main GUI application (DijkstraGUI)
│   ├── gui_canvas.py   # Canvas rendering — terrain zones, obstacles, path
│   └── gui_config.py   # Colors, grid settings, terrain zone definitions
└── README.md
```

---

## Requirements

- Python 3.8+
- numpy
- OpenCV (`cv2`) — for CLI video export only
- tkinter — included with Python on Windows/Mac

Install dependencies:
```bash
pip install numpy opencv-python
```

---

## How to Run

### GUI Mode (Recommended)

```bash
cd Code
python gui.py
```

**Steps:**
1. The map loads automatically — terrain bands and obstacles are visible.
2. **Left-click** anywhere on the free space to set the **start point** (teal circle).
3. **Right-click** to set the **goal point** (amber circle).
4. Adjust **Robot Radius**, **Clearance**, and **Step Size** with the sliders.
5. Click **RUN** — watch Dijkstra explore (blue) then draw the optimal path (teal).
6. Read **Weighted Distance** and **Nodes Explored** in the status bar.
7. Click **RESET** to clear and try a new scenario.

**Tip:** Place start and goal in different terrain zones and observe how the path bends toward the road bands to minimise cost.

### Command-Line Mode

```bash
cd Code
python dijkstra.py
```

You will be prompted for:
- Start coordinates (x, y)
- Goal coordinates (x, y)
- Robot radius
- Clearance
- Step size

Output: optimal weighted distance, nodes explored, path length, and a video file `dijkstra_path.avi`.

---

## Obstacle Map

The map is a 300×200 grid containing:

| Obstacle | Shape |
|----------|-------|
| Circle | Centre (150, 225), radius 25 |
| Ellipse | Centre (100, 150), axes 20 × 40 |
| Triangle 1 | Vertices approx. (120,20), (150,50), (185,25) |
| Triangle 2 | Vertices approx. (150,50), (185,25), (185,75) |
| Rhombus | Centre (25, 225) |
| Rectangle | Diamond-square shape near (150,75) |
| Rod | Diagonal bar near (60,65) |

Robot radius and clearance inflate each obstacle boundary during validation.

---

## Screenshots

*(Add screenshots here after running the GUI)*

---

## Key Words

Dijkstra's Algorithm · Graph-Based Navigation · Weighted Path Planning · Autonomous Vehicle · State-Space Exploration · Terrain Cost Map

---

## Software Requirements

| Library | Version | Purpose |
|---------|---------|---------|
| Python | 3.8+ | Runtime |
| numpy | any | Numerical operations |
| opencv-python | any | CLI video export |
| tkinter | built-in | GUI framework |
