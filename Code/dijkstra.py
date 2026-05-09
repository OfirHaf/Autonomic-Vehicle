# Command-line entry point for Dijkstra path planning.
# Prompts the user for start/goal coordinates, robot params, runs the search,
# and exports an AVI animation of the exploration + path.
#
# Usage:
#   cd Code
#   python dijkstra.py
#
# All heavy logic lives in utils.py (Dijkstra class).
# This file is safe to import — all executable code is guarded by
# `if __name__ == "__main__":` so importing it does NOT run input() calls.

from utils import Dijkstra

if __name__ == "__main__":
    # ---------------------------------------------------------------
    # 1. Collect user inputs
    #    Coordinates are in (col, row) order on input to match the
    #    conventional (x, y) screen notation users expect, then
    #    converted to internal (row, col) representation.
    # ---------------------------------------------------------------
    startCol  = int(input("Enter the x-coordinate for start node : "))
    startRow  = int(input("Enter the y-coordinate for start node : "))
    goalCol   = int(input("Enter the x-coordinate for goal node  : "))
    goalRow   = int(input("Enter the y-coordinate for goal node  : "))
    radius    = int(input("Enter the radius for the robot        : "))
    clearance = int(input("Enter the clearance for the robot     : "))
    stepSize  = int(input("Enter the step size                   : "))

    # Convert (col, row) user input → internal (row, col) tuples
    start = (startRow, startCol)
    goal  = (goalRow,  goalCol)

    # ---------------------------------------------------------------
    # 2. Instantiate Dijkstra and validate inputs before running
    # ---------------------------------------------------------------
    dijkstra = Dijkstra(start, goal, clearance, radius, stepSize)

    if not dijkstra.IsValid(start[0], start[1]):
        # Start is outside the drivable boundary (accounting for margin)
        print("The entered start node is outside the map.")
        print("Please check README.md for valid coordinate ranges.")
    elif not dijkstra.IsValid(goal[0], goal[1]):
        # Goal is outside the drivable boundary
        print("The entered goal node is outside the map.")
        print("Please check README.md for valid coordinate ranges.")
    elif dijkstra.IsObstacle(start[0], start[1]):
        # Start falls inside one of the seven inflated obstacles
        print("The entered start node is an obstacle.")
        print("Please choose a free-space cell.")
    elif dijkstra.IsObstacle(goal[0], goal[1]):
        # Goal falls inside one of the seven inflated obstacles
        print("The entered goal node is an obstacle.")
        print("Please choose a free-space cell.")
    elif start == goal:
        # Trivial case — no planning needed
        print("Start and goal are the same point — no path needed.")
    else:
        # ---------------------------------------------------------------
        # 3. Run Dijkstra search
        #    Returns:
        #      explored  — list of all nodes popped from the heap (in order)
        #      backtrack — optimal path from start to goal (reversed during search)
        #      distance  — total terrain-weighted path cost
        # ---------------------------------------------------------------
        explored, backtrack, distance = dijkstra.search()

        # Export exploration + path as an AVI video (requires OpenCV)
        dijkstra.animate(explored, backtrack, "./dijkstra_path.avi")

        # ---------------------------------------------------------------
        # 4. Report results
        # ---------------------------------------------------------------
        if distance == float('inf'):
            # Heap was exhausted before the goal was reached — no path exists
            print("\nNo path found between the given start and goal.")
        else:
            print(f"\nOptimal path found. Weighted distance: {distance:.4f}")
            print(f"Nodes explored: {len(explored)}")
            print(f"Path length   : {len(backtrack)} nodes")
