import math
import numpy as np
import cv2
from collections import defaultdict
from heapq import heappush, heappop


# ---------------------------------------------------------------------------
# Terrain cost zones — AV-specific feature
#
# A real autonomous vehicle prefers paved roads over rough terrain.
# We model three cost zones on the same 300×200 grid:
#
#   ROAD     (cost 1.0)  — low-cost paved lanes running horizontally
#   NORMAL   (cost 2.5)  — open space / parking areas
#   OFFROAD  (cost 6.0)  — rough terrain / sidewalks the car avoids
#
# Zone boundaries are defined by row ranges (Y-axis of the grid).
# gui_config.py IMPORTS these lists — never duplicate them there —
# so the visual canvas shading and the algorithmic cost map are always in sync.
# ---------------------------------------------------------------------------
ROAD_ROWS    = [(30, 50), (90, 110), (150, 170)]            # horizontal road bands (rows inclusive)
OFFROAD_ROWS = [(1, 20), (60, 80), (120, 140), (180, 200)]  # rough / off-road zones

COST_ROAD    = 1.0   # cheapest — AV actively seeks these rows
COST_NORMAL  = 2.5   # moderate cost (parking lots, open plazas)
COST_OFFROAD = 6.0   # expensive — AV avoids unless necessary

# Pre-compute √2 once for diagonal edge weights (avoids repeated math.sqrt calls)
_SQRT2 = math.sqrt(2)


def terrain_cost(row):
    """Return the movement cost multiplier for a given grid row.

    Checked in priority order: road first (cheapest), then off-road,
    then everything else defaults to COST_NORMAL.
    """
    for lo, hi in ROAD_ROWS:
        if lo <= row <= hi:
            return COST_ROAD        # paved road — lowest cost
    for lo, hi in OFFROAD_ROWS:
        if lo <= row <= hi:
            return COST_OFFROAD     # rough terrain — highest cost
    return COST_NORMAL              # everything else


# ---------------------------------------------------------------------------
# Dijkstra class
#
# Key difference from A*: NO heuristic (costToGo is always 0).
# Dijkstra expands nodes in strict order of cumulative cost from the start,
# guaranteeing the globally optimal weighted path.
#
# Implementation notes:
#  - defaultdict avoids pre-allocating 60 000 entries up front (was 3 × 60k)
#  - Lazy-deletion min-heap: stale entries are left in the heap and skipped
#    when popped (checked via self.visited). This is simpler than a true
#    decrease-key and equally correct for this grid size.
#  - 8-connected grid: 4 cardinal + 4 diagonal moves per cell
# ---------------------------------------------------------------------------
class Dijkstra(object):

    def __init__(self, start, goal, clearance, radius, stepSize):
        self.start     = start       # (row, col) tuple — internal representation
        self.goal      = goal        # (row, col) tuple
        self.numRows   = 200         # grid height (row 1 = bottom, row 200 = top)
        self.numCols   = 300         # grid width
        self.stepSize  = stepSize    # movement granularity in grid cells
        self.clearance = clearance   # safety margin added around each obstacle
        self.radius    = radius      # robot body radius (treated as obstacle inflation)

        # defaultdict means accessing a missing key returns the sentinel
        # automatically — no need to pre-populate the entire grid.
        self.costToCome = defaultdict(lambda: float('inf'))  # g(n): best known cost from start
        self.visited    = defaultdict(bool)                  # True once a node is finalized
        self.path       = defaultdict(lambda: -1)            # parent pointer for backtracking

    # ------------------------------------------------------------------
    # Boundary check
    # Ensures (row, col) is inside the usable grid, leaving a margin of
    # (radius + clearance) cells around every edge.
    # ------------------------------------------------------------------
    def IsValid(self, currRow, currCol):
        margin = self.radius + self.clearance
        return (
            currRow >= (1 + margin) and
            currRow <= (self.numRows - margin) and
            currCol >= (1 + margin) and
            currCol <= (self.numCols - margin)
        )

    # ------------------------------------------------------------------
    # Obstacle map — seven analytic shapes
    #
    # Each shape is defined by its implicit boundary equation(s).
    # c_r = clearance + radius inflates every obstacle so the robot body
    # (modelled as a circle of radius `radius`) plus the safety `clearance`
    # never touches the true obstacle boundary.
    #
    # Shapes:
    #   dist1 — circle       (standard Euclidean distance)
    #   dist2 — ellipse      (normalised Euclidean)
    #   dist3 — triangle 1   (half-plane intersection, all three sides ≤ 0)
    #   dist4 — triangle 2   (half-plane intersection, all three sides ≥ 0)
    #   dist5 — rhombus      (four half-planes, all ≥ 0)
    #   dist6 — rectangle    (four half-planes, all ≤ 0)
    #   dist7 — rod          (diagonal bar — four half-planes with mixed signs)
    # ------------------------------------------------------------------
    def IsObstacle(self, row, col):
        c_r     = self.clearance + self.radius   # total inflation per obstacle
        sqrt_cr = _SQRT2 * c_r                   # diagonal component of inflation

        # ---- Circle -------------------------------------------------------
        # Inside when squared distance from centre ≤ (radius + inflation)²
        dist1 = (row - 150) ** 2 + (col - 225) ** 2 - (25 + c_r) ** 2

        # ---- Ellipse -------------------------------------------------------
        # Standard ellipse equation; ≤ 0 means inside the inflated ellipse
        dist2 = (
            ((row - 100) ** 2) / ((20 + c_r) ** 2) +
            ((col - 150) ** 2) / ((40 + c_r) ** 2)
        ) - 1

        # ---- Triangle 1 (upper-right corner group) -------------------------
        # Point is inside when it is on the correct side of all three edges.
        # Vertices are offset by c_r in the inward-normal direction.
        x1, y1 = 120 - 2.62 * c_r, 20 - 1.205 * c_r
        x2, y2 = 150 - sqrt_cr,     50
        x3, y3 = 185 + c_r,         25 - c_r * 0.9247
        f1 = (col - y1) * (x2 - x1) - (y2 - y1) * (row - x1)
        f2 = (col - y2) * (x3 - x2) - (y3 - y2) * (row - x2)
        f3 = (col - y3) * (x1 - x3) - (y1 - y3) * (row - x3)
        dist3 = 0 if (f1 <= 0 and f2 <= 0 and f3 <= 0) else 1  # 0 = inside

        # ---- Triangle 2 (lower-right, shares base with triangle 1) ---------
        x1, y1 = 150 - sqrt_cr, 50
        x2, y2 = 185 + c_r,     25 - c_r * 0.9247
        x3, y3 = 185 + c_r,     75 + c_r * 0.714
        f1 = (col - y1) * (x2 - x1) - (y2 - y1) * (row - x1)
        f2 = (col - y2) * (x3 - x2) - (y3 - y2) * (row - x2)
        f3 = (col - y3) * (x1 - x3) - (y1 - y3) * (row - x3)
        dist4 = 0 if (f1 >= 0 and f2 >= 0 and f3 >= 0) else 1

        # ---- Rhombus (diamond near left edge) ------------------------------
        # Four corners; inside when all four half-plane tests are ≥ 0.
        x1, y1 = 10 - sqrt_cr,  225
        x2, y2 = 25,             200 - sqrt_cr
        x3, y3 = 40 + sqrt_cr,  225
        x4, y4 = 25,             250 + sqrt_cr
        f1 = (col - y1) * (x2 - x1) - (y2 - y1) * (row - x1)
        f2 = (col - y2) * (x3 - x2) - (y3 - y2) * (row - x2)
        f3 = (col - y3) * (x4 - x3) - (y4 - y3) * (row - x3)
        f4 = (col - y4) * (x1 - x4) - (y1 - y4) * (row - x4)
        dist5 = 0 if (f1 >= 0 and f2 >= 0 and f3 >= 0 and f4 >= 0) else 1

        # ---- Rectangle (tilted, near centre) --------------------------------
        # Inside when all four half-plane tests are ≤ 0.
        x1, y1 = 150 - sqrt_cr, 50
        x2, y2 = 120 - sqrt_cr, 75
        x3, y3 = 150,            100 + sqrt_cr
        x4, y4 = 185 + c_r,     75 + c_r * 0.714
        f1 = (col - y1) * (x2 - x1) - (y2 - y1) * (row - x1)
        f2 = (col - y2) * (x3 - x2) - (y3 - y2) * (row - x2)
        f3 = (col - y3) * (x4 - x3) - (y4 - y3) * (row - x3)
        f4 = (col - y4) * (x1 - x4) - (y1 - y4) * (row - x4)
        dist6 = 0 if (f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0) else 1

        # ---- Rod (diagonal bar across lower-left region) --------------------
        # Defined by four bounding half-planes; specific sign pattern marks interior.
        f1 = (col - 95)               * (8.66  + sqrt_cr) - (5      + sqrt_cr) * (row - 30 + sqrt_cr)
        f2 = (col - 95)               * (37.5  + sqrt_cr) - (-64.95 - sqrt_cr) * (row - 30 + sqrt_cr)
        f3 = (col - 30.05 + sqrt_cr)  * (8.65  + sqrt_cr) - (5.45   + sqrt_cr) * (row - 67.5)
        f4 = (col - 35.5)             * (-37.49 - sqrt_cr) - (64.5  + sqrt_cr) * (row - 76.15 - sqrt_cr)
        dist7 = 0 if (f1 <= 0 and f2 >= 0 and f3 >= 0 and f4 >= 0) else 1

        # A cell is an obstacle if it is inside ANY of the seven shapes
        return (
            dist1 <= 0 or dist2 <= 0 or
            dist3 == 0 or dist4 == 0 or dist5 == 0 or dist6 == 0 or dist7 == 0
        )

    # ------------------------------------------------------------------
    # Movement validity helpers
    # A move is valid only if the destination cell is:
    #   1. Within the boundary margin (IsValid)
    #   2. Not inside any inflated obstacle (IsObstacle)
    #   3. Not already finalized in this search (self.visited)
    # ------------------------------------------------------------------
    def _can_move(self, row, col):
        return (
            self.IsValid(row, col) and
            not self.IsObstacle(row, col) and
            not self.visited[(row, col)]
        )

    # Eight directional move checks — stepSize allows coarser resolution
    def ActionMoveLeft(self, r, c):      return self._can_move(r, c - self.stepSize)
    def ActionMoveRight(self, r, c):     return self._can_move(r, c + self.stepSize)
    def ActionMoveUp(self, r, c):        return self._can_move(r - self.stepSize, c)
    def ActionMoveDown(self, r, c):      return self._can_move(r + self.stepSize, c)
    def ActionMoveRightUp(self, r, c):   return self._can_move(r - self.stepSize, c + self.stepSize)
    def ActionMoveRightDown(self, r, c): return self._can_move(r + self.stepSize, c + self.stepSize)
    def ActionMoveLeftUp(self, r, c):    return self._can_move(r - self.stepSize, c - self.stepSize)
    def ActionMoveLeftDown(self, r, c):  return self._can_move(r + self.stepSize, c - self.stepSize)

    # ------------------------------------------------------------------
    # Edge weight calculation
    #
    # weight = base_distance × average_terrain_cost(from_row, to_row) × stepSize
    #
    # Averaging the terrain cost between the source and destination cell
    # gives a smoother cost landscape at zone boundaries — the robot is
    # penalised proportionally for crossing from cheap to expensive terrain.
    # ------------------------------------------------------------------
    def _edge_cost(self, from_row, to_row, diagonal=False):
        base = _SQRT2 if diagonal else 1.0          # diagonal moves are √2 longer
        cost = 0.5 * (terrain_cost(from_row) + terrain_cost(to_row))  # average terrain
        return base * cost * self.stepSize

    # ------------------------------------------------------------------
    # Edge relaxation — core Dijkstra operation
    #
    # If a cheaper path to (new_row, new_col) is discovered via `current`,
    # update the cost and parent pointer, and signal that a heap push is needed.
    # ------------------------------------------------------------------
    def _relax(self, current, new_row, new_col, diagonal=False):
        new_cost = self.costToCome[current] + self._edge_cost(
            current[0], new_row, diagonal
        )
        if new_cost < self.costToCome[(new_row, new_col)]:
            self.costToCome[(new_row, new_col)] = new_cost
            self.path[(new_row, new_col)] = current   # update parent for backtracking
            return True    # caller should push (new_cost, (new_row, new_col)) to heap
        return False

    # ------------------------------------------------------------------
    # Dijkstra search — main loop
    #
    # Returns a 3-tuple:
    #   exploredStates  — ordered list of all nodes finalized (for animation)
    #   backtrackStates — optimal path from start → goal (empty if no path)
    #   cost            — total terrain-weighted distance (inf if no path)
    # ------------------------------------------------------------------
    def search(self):
        explored_states = []
        queue = []

        # Initialise: start node costs 0
        self.costToCome[self.start] = 0.0
        heappush(queue, (0.0, self.start))  # (cost, node) — heapq uses the first element as key

        while queue:
            cost, current = heappop(queue)

            # Lazy-deletion: if this node was already finalized, skip the stale entry
            if self.visited[current]:
                continue
            self.visited[current] = True
            explored_states.append(current)

            # Goal reached — Dijkstra guarantees this is the optimal path
            if current == self.goal:
                break

            r, c = current

            # --- Straight moves (cost multiplier = 1.0) ---
            moves_straight = []
            if self.ActionMoveLeft(r, c):      moves_straight.append((r, c - self.stepSize))
            if self.ActionMoveRight(r, c):     moves_straight.append((r, c + self.stepSize))
            if self.ActionMoveUp(r, c):        moves_straight.append((r - self.stepSize, c))
            if self.ActionMoveDown(r, c):      moves_straight.append((r + self.stepSize, c))

            for nr, nc in moves_straight:
                if self._relax(current, nr, nc, diagonal=False):
                    heappush(queue, (self.costToCome[(nr, nc)], (nr, nc)))

            # --- Diagonal moves (cost multiplier = √2) ---
            moves_diag = []
            if self.ActionMoveRightUp(r, c):   moves_diag.append((r - self.stepSize, c + self.stepSize))
            if self.ActionMoveRightDown(r, c): moves_diag.append((r + self.stepSize, c + self.stepSize))
            if self.ActionMoveLeftUp(r, c):    moves_diag.append((r - self.stepSize, c - self.stepSize))
            if self.ActionMoveLeftDown(r, c):  moves_diag.append((r + self.stepSize, c - self.stepSize))

            for nr, nc in moves_diag:
                if self._relax(current, nr, nc, diagonal=True):
                    heappush(queue, (self.costToCome[(nr, nc)], (nr, nc)))

        # ------------------------------------------------------------------
        # Backtrack — reconstruct path by following parent pointers
        # ------------------------------------------------------------------
        if self.costToCome[self.goal] == float('inf'):
            # Goal was never reached — heap exhausted without finding the goal
            return (explored_states, [], float('inf'))

        backtrack = []
        node = self.goal
        while self.path[node] != -1:     # walk back until we hit the start (parent == -1)
            backtrack.append(node)
            node = self.path[node]
        backtrack.append(self.start)     # include the start node itself
        backtrack.reverse()              # reverse so path goes start → goal

        return (explored_states, backtrack, self.costToCome[self.goal])

    # ------------------------------------------------------------------
    # Video animation (CLI mode only)
    #
    # Renders each explored node in yellow frame-by-frame, then draws
    # the final path in red. Outputs an AVI file via OpenCV VideoWriter.
    #
    # Note: cv2.imshow requires a display (not available on headless servers).
    # ------------------------------------------------------------------
    def animate(self, explored_states, backtrack_states, path):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out    = cv2.VideoWriter(str(path), fourcc, 20.0, (self.numCols, self.numRows))

        # Black canvas; terrain zones are painted as coloured horizontal bands
        image  = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)

        # Draw terrain background — road = dark green, off-road = dark blue
        for row in range(1, self.numRows + 1):
            cost = terrain_cost(row)
            if cost == COST_ROAD:
                image[self.numRows - row, :] = (40, 80, 40)    # BGR: dark green
            elif cost == COST_OFFROAD:
                image[self.numRows - row, :] = (20, 20, 60)    # BGR: dark blue

        # Exploration phase — paint each explored node yellow, write every 80th frame
        for count, state in enumerate(explored_states):
            image[self.numRows - state[0], state[1] - 1] = (255, 255, 0)  # BGR yellow
            if count % 80 == 0:
                out.write(image)
                cv2.imshow('result', image)
                cv2.waitKey(1)   # 1 ms delay keeps the window responsive

        # Path phase — paint the final optimal path in red (BGR: 0, 0, 255)
        if backtrack_states:
            for state in backtrack_states:
                image[self.numRows - state[0], state[1] - 1] = (0, 0, 255)
                out.write(image)
                cv2.imshow('result', image)
                cv2.waitKey(5)   # slightly longer delay for the path drawing

        cv2.waitKey(0)           # hold final frame until user closes window
        cv2.destroyAllWindows()
        out.release()            # flush and close the video file
