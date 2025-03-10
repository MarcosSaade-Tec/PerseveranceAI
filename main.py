import numpy as np
from simpleai.search import SearchProblem, astar, breadth_first, depth_first

mars_map = np.load('mars_map.npy')
nr, nc = mars_map.shape # 1815x756

SCALE = 10.0174

def coordinate_to_rowcol(coordinate):
    """Takes a (x_coordinate, y_coordinate) tuple and returns a (row, col) tuple

    Args:
        coordinate (tuple): Coordinates where the robot is located
    """
    x = coordinate[0]
    y = coordinate[1]

    col = round(x/SCALE)
    row = nr - round(y/SCALE)

    return col, row


# Start position and goal in coordinates
initial_pos_coordinates = (2850, 6400)

goal_coordinates = (3150, 6800)

# Convert to row, col
initial_pos = coordinate_to_rowcol(initial_pos_coordinates)
goal = coordinate_to_rowcol(goal_coordinates)


col = initial_pos[0]
row = initial_pos[1]


class PerseveranceRover(SearchProblem):
    def __init__(self):
        super(PerseveranceRover, self).__init__(initial_state=initial_pos)

    def is_goal(self, state):
        return (state[0]==goal[0] and state[1]==goal[1])

    def actions(self, state):
        actions = []

        col = state[0]
        row = state[1]

        current = mars_map[row][col]

        up = mars_map[row-1][col]
        right = mars_map[row][col+1]
        left = mars_map[row][col-1]
        down = mars_map[row+1][col]

        if right - current < 0.25 and right != -1:
            actions.append('r')
        if left - current < 0.25 and left != -1:
            actions.append('l')
        if up - current < 0.25 and up != -1:
            actions.append('u')
        if down - current < 0.25 and down != -1:
            actions.append('d')

        return tuple(actions)

    def result(self, state, action):
        state = list(state)

        if action == 'r':
            state[0] += 1
        elif action == 'l':
            state[0] -= 1
        elif action == 'u':
            state[1] -= 1
        elif action == 'd':
            state[1] += 1

        return tuple(state)

    def initial_state(self):
        return initial_pos
    
    def cost(self, state, action, state2):
        return 1

    def heuristic(self, state):
        """ 
            Heuristic is the distance to the goal
        """
        
        distance = 0

        distance += abs(state[0] - goal[0])
        distance += abs(state[1] - goal[1])

        return distance
    

### Los algoritmos de busqieda no informada no resuelven el problema en un tiempo razonable ###
# depth_first_search = depth_first(PerseveranceRover())
# print(depth_first_search.state)

# breadth_first_search = breadth_first(PerseveranceRover())
# print(breadth_first_search.state)


astar_search = astar(PerseveranceRover())
print(astar_search.state)
print(astar_search.path())
