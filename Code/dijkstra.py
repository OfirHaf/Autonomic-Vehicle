# Command-line entry point for Dijkstra path planning
# Usage: python dijkstra.py
# Mirrors the interface of astar.py from the example project.

from utils import Dijkstra

startCol = int(input("Enter the x-coordinate for start node : "))
startRow = int(input("Enter the y-coordinate for start node : "))
goalCol  = int(input("Enter the x-coordinate for goal node  : "))
goalRow  = int(input("Enter the y-coordinate for goal node  : "))
radius   = int(input("Enter the radius for the robot        : "))
clearance = int(input("Enter the clearance for the robot    : "))
stepSize  = int(input("Enter the step size                  : "))

start = (startRow, startCol)
goal  = (goalRow,  goalCol)

dijkstra = Dijkstra(start, goal, clearance, radius, stepSize)

if not dijkstra.IsValid(start[0], start[1]):
    print("The entered start node is outside the map.")
    print("Please check README.md for valid coordinate ranges.")
elif not dijkstra.IsValid(goal[0], goal[1]):
    print("The entered goal node is outside the map.")
    print("Please check README.md for valid coordinate ranges.")
elif dijkstra.IsObstacle(start[0], start[1]):
    print("The entered start node is an obstacle.")
    print("Please choose a free-space cell.")
elif dijkstra.IsObstacle(goal[0], goal[1]):
    print("The entered goal node is an obstacle.")
    print("Please choose a free-space cell.")
else:
    explored, backtrack, distance = dijkstra.search()
    dijkstra.animate(explored, backtrack, "./dijkstra_path.avi")

    if distance == float('inf'):
        print("\nNo path found between the given start and goal.")
    else:
        print(f"\nOptimal path found. Weighted distance: {distance:.4f}")
        print(f"Nodes explored: {len(explored)}")
        print(f"Path length   : {len(backtrack)} nodes")
