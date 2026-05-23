# Project Summary — Path Planning for Autonomous Vehicles

## What the Project Does

This project implements **Dijkstra's shortest-path algorithm** for an Autonomous Vehicle (AV) navigating a 300 × 200 grid map containing seven static obstacles and three terrain cost zones (road × 1.0, open space × 2.5, off-road × 6.0). The AV finds the globally optimal weighted path from any user-defined start to any goal, preferring paved road bands even when geometrically longer detours are required.

## How to Run

```bash
pip install numpy opencv-python
cd Code
python gui.py          # interactive GUI (recommended)
python dijkstra.py     # command-line with video export
```

## Key Technical Contributions

| Feature | Detail |
|---------|--------|
| Algorithm | Dijkstra with lazy-deletion min-heap |
| Connectivity | 8-connected grid (straight + diagonal moves) |
| Terrain model | 3 cost zones synced between algorithm and renderer |
| Obstacle inflation | Analytic c_r = radius + clearance per shape |
| GUI | Tkinter with real-time exploration animation |
| CLI export | OpenCV AVI video of exploration + path |

## Results (step size = 1, radius = 0, clearance = 0)

- **Weighted distance:** ~215 units (terrain-weighted)
- **Nodes explored:** 14,000 – 28,000
- **Path length:** 180 – 260 nodes
- **Runtime:** < 3 seconds

## Deliverables

| # | File | Description |
|---|------|-------------|
| 1 | `AV_Dijkstra_Presentation.pptx` | 12-slide PowerPoint presentation |
| 2 | `AV_Dijkstra_Theory_Doc.docx` | 10-section Word theory document (6+ pages) |
| 3 | *(link in slide 12)* | Presentation recording — **add your video URL** |
| 4 | https://github.com/OfirHaf/Autonomic-Vehicle | GitHub repository |
| 5 | `Project_Summary.md` | This file |

## Project Structure

```
Autonomic-Vehicle/
├── Code/
│   ├── dijkstra.py      CLI entry point
│   ├── utils.py         Dijkstra class + terrain model
│   ├── gui.py           Main GUI application
│   ├── gui_canvas.py    Canvas renderer
│   └── gui_config.py    Colors, config, terrain zones
├── AV_Dijkstra_Presentation.pptx
├── AV_Dijkstra_Theory_Doc.docx
├── Project_Summary.md
└── README.md
```

## What to Add Before Submission

1. **GitHub link** — https://github.com/OfirHaf/Autonomic-Vehicle
2. **Recording link** — record a 5–12 min walkthrough (OBS, Loom, or screen-record) showing the GUI running, then add the link on slide 12.
