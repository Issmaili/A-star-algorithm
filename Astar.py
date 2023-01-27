import heapq
import random

# Helper function to calculate the heuristic value
def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

# A* algorithm
def a_star(landscape, start, goal, obstacles):
    # Initialize open and closed lists
    open_list = []
    closed_list = set()

    # Add the start node to the open list
    heapq.heappush(open_list, (0, start))

    # Store the came from information for each node
    came_from = {}

    # Store the cost of moving to each node
    g_score = {start: 0}

    # Keep track of the number of obstacles
    obstacles = set(obstacles)

    while open_list:
        # Get the node with the lowest f-value from the open list
        current = heapq.heappop(open_list)[1]

        # Check if we have reached the goal
        if current == goal:
            # Reconstruct the path and return it
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        # Add the current node to the closed list
        closed_list.add(current)

        # Expand the neighbors of the current node
        for neighbor in [(current[0] + dx, current[1] + dy) for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]]:
            # Check if the neighbor is out of the landscape or an obstacle
            if (not (0 <= neighbor[0] < len(landscape))) or (not (0 <= neighbor[1] < len(landscape[0]))) or (neighbor in obstacles):
                continue

            # Calculate the cost of moving to the neighbor
            new_g_score = g_score[current] + 1

            # Check if the neighbor is already in the closed list
            if neighbor in closed_list and new_g_score >= g_score.get(neighbor, float('inf')):
                continue

            # Update the came from information and cost of moving to the neighbor
            came_from[neighbor] = current
            g_score[neighbor] = new_g_score

            # Calculate the f-value of the neighbor
            f_value = new_g_score + heuristic(neighbor, goal)

            # Add the neighbor to the open list
            heapq.heappush(open_list, (f_value, neighbor))

    # If the open list is empty, then there is no path from the start to the goal
    return None

# Example usage
n = 5
m = 4
landscape = [[0 for _ in range(m)] for _ in range(n)]
start = (0, 0)
goal = (n-1, m-1)
num_obstacles = int(input("Enter the number of obstacles: "))
# obstacles = random.sample

# generate random obstacles
obstacles = random.sample([(i, j) for i in range(n) for j in range(m)], num_obstacles)
for obstacle in obstacles:
    landscape[obstacle[0]][obstacle[1]] = 1

# run the A* algorithm
path = a_star(landscape, start, goal, obstacles)
if path:
    print("Shortest path:", path)
else:
    print("No path found.")

print("Matrica:", landscape)

