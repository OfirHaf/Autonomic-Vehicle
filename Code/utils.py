# header files
import numpy as np
import cv2
from heapq import heappush, heappop


# ---------------------------------------------------------------------------
# Terrain cost zones — AV-specific feature
# A real autonomous vehicle prefers paved roads over rough terrain.
# We model three cost zones on the same 300x200 grid:
#   ROAD     (cost 1.0)  — low-cost paved lanes running horizontally
#   NORMAL   (cost 2.5)  — open space / parking areas
#   OFFROAD  (cost 6.0)  — rough terrain / sidewalks the car avoids
#
# Zone boundaries are defined by row ranges (Y-axis of the grid).
# ---------------------------------------------------------------------------
ROAD_ROWS   = [(30, 50), (90, 110), (150, 170)]   # horizontal road bands
OFFROAD_ROWS = [(1, 20), (60, 80), (120, 140), (180, 200)]  # rough zones

COST_ROAD    = 1.0
COST_NORMAL  = 2.5
COST_OFFROAD = 6.0


def terrain_cost(row):
    """Return movement cost multiplier for a given grid row."""
    for lo, hi in ROAD_ROWS:
        if lo <= row <= hi:
            return COST_ROAD
    for lo, hi in OFFROAD_ROWS:
        if lo <= row <= hi:
            return COST_OFFROAD
    return COST_NORMAL


# ---------------------------------------------------------------------------
# Dijkstra class — mirrors AStar class structure from the example project
# Key difference: NO heuristic (costToGo is always 0).
# Dijkstra expands nodes in order of cumulative cost from the start,
# guaranteeing the optimal path in a weighted graph.
# ---------------------------------------------------------------------------
class Dijkstra(object):

    def __init__(self, start, goal, clearance, radius, stepSize):
        self.start    = start
        self.goal     = goal
        self.numRows  = 200
        self.numCols  = 300
        self.stepSize = stepSize
        self.clearance = clearance
        self.radius    = radius

        # Per-node bookkeeping (same dict structure as AStar for consistency)
        self.costToCome = {}
        self.path       = {}
        self.visited    = {}

        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                self.visited[(row, col)]    = False
                self.path[(row, col)]       = -1
                self.costToCome[(row, col)] = float('inf')

    # ------------------------------------------------------------------
    # Boundary check — same formula as AStar
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
    # Obstacle map — identical to the A* example so the two projects
    # run on the same environment and can be visually compared.
    # Obstacles: circle, ellipse, triangles, rhombus, rectangle, rod.
    # ------------------------------------------------------------------
    def IsObstacle(self, row, col):
        c_r = self.clearance + self.radius
        sqrt_cr = 1.4142 * c_r

        # Circle
        dist1 = (
            (row - 150) ** 2 + (col - 225) ** 2
        ) - (25 + c_r) ** 2

        # Ellipse
        dist2 = (
            ((row - 100) ** 2) / ((20 + c_r) ** 2) +
            ((col - 150) ** 2) / ((40 + c_r) ** 2)
        ) - 1

        # Triangle 1
        x1, y1 = 120 - 2.62 * c_r,  20 - 1.205 * c_r
        x2, y2 = 150 - sqrt_cr,      50
        x3, y3 = 185 + c_r,          25 - c_r * 0.9247
        f1 = (col - y1) * (x2 - x1) - (y2 - y1) * (row - x1)
        f2 = (col - y2) * (x3 - x2) - (y3 - y2) * (row - x2)
        f3 = (col - y3) * (x1 - x3) - (y1 - y3) * (row - x3)
        dist3 = 0 if (f1 <= 0 and f2 <= 0 and f3 <= 0) else 1

        # Triangle 2
        x1, y1 = 150 - sqrt_cr,  50
        x2, y2 = 185 + c_r,      25 - c_r * 0.9247
        x3, y3 = 185 + c_r,      75 + c_r * 0.714
        f1 = (col - y1) * (x2 - x1) - (y2 - y1) * (row - x1)
        f2 = (col - y2) * (x3 - x2) - (y3 - y2) * (row - x2)
        f3 = (col - y3) * (x1 - x3) - (y1 - y3) * (row - x3)
        dist4 = 0 if (f1 >= 0 and f2 >= 0 and f3 >= 0) else 1

        # Rhombus
        x1, y1 = 10 - sqrt_cr,  225
        x2, y2 = 25,             200 - sqrt_cr
        x3, y3 = 40 + sqrt_cr,  225
        x4, y4 = 25,             250 + sqrt_cr
        f1 = (col - y1) * (x2 - x1) - (y2 - y1) * (row - x1)
        f2 = (col - y2) * (x3 - x2) - (y3 - y2) * (row - x2)
        f3 = (col - y3) * (x4 - x3) - (y4 - y3) * (row - x3)
        f4 = (col - y4) * (x1 - x4) - (y1 - y4) * (row - x4)
        dist5 = 0 if (f1 >= 0 and f2 >= 0 and f3 >= 0 and f4 >= 0) else 1

        # Rectangle (diamond-square)
        x1, y1 = 150 - sqrt_cr,  50
        x2, y2 = 120 - sqrt_cr,  75
        x3, y3 = 150,             100 + sqrt_cr
        x4, y4 = 185 + c_r,      75 + c_r * 0.714
        f1 = (col - y1) * (x2 - x1) - (y2 - y1) * (row - x1)
        f2 = (col - y2) * (x3 - x2) - (y3 - y2) * (row - x2)
        f3 = (col - y3) * (x4 - x3) - (y4 - y3) * (row - x3)
        f4 = (col - y4) * (x1 - x4) - (y1 - y4) * (row - x4)
        dist6 = 0 if (f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0) else 1

        # Rod
        f1 = (col - 95)          * (8.66  + sqrt_cr) - (5    + sqrt_cr) * (row - 30 + sqrt_cr)
        f2 = (col - 95)          * (37.5  + sqrt_cr) - (-64.95 - sqrt_cr) * (row - 30 + sqrt_cr)
        f3 = (col - 30.05 + sqrt_cr) * (8.65 + sqrt_cr) - (5.45 + sqrt_cr) * (row - 67.5)
        f4 = (col - 35.5)        * (-37.49 - sqrt_cr) - (64.5 + sqrt_cr) * (row - 76.15 - sqrt_cr)
        dist7 = 0 if (f1 <= 0 and f2 >= 0 and f3 >= 0 and f4 >= 0) else 1

        return not all(d != 0 for d in [dist1, dist2, dist3, dist4, dist5, dist6, dist7]) or \
               dist1 <= 0 or dist2 <= 0 or dist3 == 0 or dist4 == 0 or \
               dist5 == 0 or dist6 == 0 or dist7 == 0

    # ------------------------------------------------------------------
    # Movement validity helpers — same pattern as AStar
    # ------------------------------------------------------------------
    def _can_move(self, row, col):
        return (
            self.IsValid(row, col) and
            not self.IsObstacle(row, col) and
            not self.visited[(row, col)]
        )

    def ActionMoveLeft(self, r, c):      return self._can_move(r, c - self.stepSize)
    def ActionMoveRight(self, r, c):     return self._can_move(r, c + self.stepSize)
    def ActionMoveUp(self, r, c):        return self._can_move(r - self.stepSize, c)
    def ActionMoveDown(self, r, c):      return self._can_move(r + self.stepSize, c)
    def ActionMoveRightUp(self, r, c):   return self._can_move(r - self.stepSize, c + self.stepSize)
    def ActionMoveRightDown(self, r, c): return self._can_move(r + self.stepSize, c + self.stepSize)
    def ActionMoveLeftUp(self, r, c):    return self._can_move(r - self.stepSize, c - self.stepSize)
    def ActionMoveLeftDown(self, r, c):  return self._can_move(r + self.stepSize, c - self.stepSize)

    # ------------------------------------------------------------------
    # Edge weight — straight: step cost × terrain, diagonal: × √2
    # Terrain cost is averaged between current and neighbour cell.
    # ------------------------------------------------------------------
    def _edge_cost(self, from_row, to_row, diagonal=False):
        base = 1.414 if diagonal else 1.0
        cost = 0.5 * (terrain_cost(from_row) + terrain_cost(to_row))
        return base * cost * self.stepSize

    # ------------------------------------------------------------------
    # Relax an edge: update costToCome if a cheaper path is found.
    # Returns True if the heap should be updated.
    # ------------------------------------------------------------------
    def _relax(self, current, new_row, new_col, diagonal=False):
        new_cost = self.costToCome[current] + self._edge_cost(
            current[0], new_row, diagonal
        )
        if new_cost < self.costToCome[(new_row, new_col)]:
            self.costToCome[(new_row, new_col)] = new_cost
            self.path[(new_row, new_col)] = current
            return True
        return False

    # ------------------------------------------------------------------
    # Dijkstra search — returns (exploredStates, backtrackStates, cost)
    # Same return signature as AStar.search() for GUI compatibility.
    # ------------------------------------------------------------------
    def search(self):
        explored_states = []
        queue = []

        self.costToCome[self.start] = 0.0
        heappush(queue, (0.0, self.start))

        while queue:
            cost, current = heappop(queue)

            # Skip stale entries (lazy deletion pattern)
            if self.visited[current]:
                continue
            self.visited[current] = True
            explored_states.append(current)

            if current == self.goal:
                break

            r, c = current

            # 4-directional straight moves
            moves_straight = []
            if self.ActionMoveLeft(r, c):      moves_straight.append((r, c - self.stepSize))
            if self.ActionMoveRight(r, c):     moves_straight.append((r, c + self.stepSize))
            if self.ActionMoveUp(r, c):        moves_straight.append((r - self.stepSize, c))
            if self.ActionMoveDown(r, c):      moves_straight.append((r + self.stepSize, c))

            for nr, nc in moves_straight:
                if self._relax(current, nr, nc, diagonal=False):
                    heappush(queue, (self.costToCome[(nr, nc)], (nr, nc)))

            # 4-directional diagonal moves
            moves_diag = []
            if self.ActionMoveRightUp(r, c):   moves_diag.append((r - self.stepSize, c + self.stepSize))
            if self.ActionMoveRightDown(r, c): moves_diag.append((r + self.stepSize, c + self.stepSize))
            if self.ActionMoveLeftUp(r, c):    moves_diag.append((r - self.stepSize, c - self.stepSize))
            if self.ActionMoveLeftDown(r, c):  moves_diag.append((r + self.stepSize, c - self.stepSize))

            for nr, nc in moves_diag:
                if self._relax(current, nr, nc, diagonal=True):
                    heappush(queue, (self.costToCome[(nr, nc)], (nr, nc)))

        # No path found
        if self.costToCome[self.goal] == float('inf'):
            return (explored_states, [], float('inf'))

        # Backtrack from goal to start
        backtrack = []
        node = self.goal
        while self.path[node] != -1:
            backtrack.append(node)
            node = self.path[node]
        backtrack.append(self.start)
        backtrack.reverse()

        return (explored_states, backtrack, self.costToCome[self.goal])

    # ------------------------------------------------------------------
    # Video animation — same as AStar.animate() for CLI mode
    # ------------------------------------------------------------------
    def animate(self, explored_states, backtrack_states, path):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(str(path), fourcc, 20.0, (self.numCols, self.numRows))
        image = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)

        # Draw terrain zones in background
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                cost = terrain_cost(row)
                if cost == COST_ROAD:
                    image[self.numRows - row, col - 1] = (40, 80, 40)
                elif cost == COST_OFFROAD:
                    image[self.numRows - row, col - 1] = (20, 20, 60)

        count = 0
        for state in explored_states:
            image[self.numRows - state[0], state[1] - 1] = (255, 255, 0)
            if count % 80 == 0:
                out.write(image)
            count += 1

        if backtrack_states:
            for state in backtrack_states:
                image[self.numRows - state[0], state[1] - 1] = (0, 0, 255)
                out.write(image)
                cv2.imshow('result', image)
                cv2.waitKey(5)

        cv2.waitKey(0)
        cv2.destroyAllWindows()
        out.release()
