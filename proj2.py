"""
File: proj2.py -- Matt Marks, Oct 29, 2019


My project works by peering into the future using a minimax algorithm by playin
the min player. The minimax tree is pruned using alpha beta pruning.
My bfs_heuristic is custom made becuase h_walldist runs very slow. Mine is a
reverse BFS that gives each point a rough path length to the goal.


I have also added the following adjustments:
- Prioritize moving to areas where there have been less moves.
- I keep track of a history of moves and use it for cycle detection. (data2.txt)
- I will never move to a state where the control error can just kill me.

"""

import racetrack_example as rt
import math
import sys
import json

# Global variable for h_walldist
infinity = float('inf')     # same as math.inf

def main(state,finish,walls):

    ((x,y),(u,v)) = state

    data_file = open('data.txt', 'r')
    heuristic = json.load(data_file)
    data_file.close()

    choices_file = open('choices.txt', 'w')

    # Where I store cycle detection.
    data2_file = open('data2.txt', 'r')
    history = json.load(data2_file)
    data2_file.close()

    selected_velocity = (0,0)
    if hit_or_near_goal(state, finish):
        print((0,0),file=choices_file,flush=True)
    else:
        best = infinity

        for possible_state in my_states(state, walls, finish):

            result = alpha_beta(possible_state, 2, -infinity, infinity, False, finish, walls, heuristic)

            if result <= best:

                should_update = True
                # We quickly check to see if we have been at
                # this state and made this decision before.
                if str(state[0]) in history:
                    if str(possible_state[1]) in history[str(state[0])]:
                        should_update = False

                # If we have nore made this decision before...
                if should_update:
                    best = result
                    selected_velocity = possible_state[1]
                    print(possible_state[1],file=choices_file,flush=True)

    # We add the chosen velocity to the history of velocities.
    if str(state[0]) not in history:
        history[str(state[0])] = []
    history[str(state[0])].append(str(selected_velocity))
    data2_file = open('data2.txt', 'w')
    json.dump(history, data2_file)
    data2_file.close()




def alpha_beta(state, depth, alpha, beta, maximising_player, fline, walls, heuristic):

    """
    Called on each possible neighboring state. Uses a minimax algorithm to
    return a value for the quality of that state. The smaller the better.
    """

    data2_file = open('data2.txt', 'r')
    history = json.load(data2_file)
    data2_file.close()

    # We first check to see if the state is in bounds.
    if str(state[0]) not in heuristic:
        return infinity

    # We also check to see if we hit the goal.
    if hit_or_near_goal(state, fline):
        return 0.0 # Lowest possible heuristic.

    # We also check to see if we have hit our depth limit.
    if depth == 0:
        return heuristic[str(state[0])]

    if maximising_player:
        value = -infinity
        for child in control_error_states(state):
            value = max(value, alpha_beta(child, depth-1, alpha, beta, False, fline, walls, heuristic))
            alpha = max(alpha, value)
            if alpha >= beta:
                return heuristic[str(state[0])]
        return value
    else:
        value = infinity
        for child in my_states(state, walls, fline):
            value = min(value, alpha_beta(child, depth-1, alpha, beta, True, fline, walls, heuristic))

            if str(child[0]) in history:
                # If we have been to this state before...
                value = infinity
            else:
                # otherwise, we make moves to places of high previous move density
                # less likley.
                density = calculate_visited_point_density(child[0], history)
                value += density ** 3

            beta = min(beta, value)
            if alpha >= beta:
                return heuristic[str(state[0])]
        return value


def bfs_heuristic(walls, fline):
    """
    Performs a reverse BFS starting at the finish line and working backwards
    to traverse the whole space.

    A dictionary is returned whose keys are the (x,y) positions and whose values
    are the path distance found by the BFS to the goal.
    (The key is cast to a string to it can be written to a file if needed.)
    """

    results = dict()
    Q = Queue()
    visited = set()
    ((fx1, fy1), (fx2, fy2)) = fline

    fx_start = min(fx1, fx2)
    fx_stop  = max(fx1, fx2)
    fy_start = min(fy1, fy2)
    fy_stop  = max(fy1, fy2)

    # We start our search from the center of the finish line.
    center_of_finish = ( (fx_start + fx_stop)//2, (fy_start + fy_stop)//2 )
    visited.add(center_of_finish)
    Q.enqueue(Node(center_of_finish, 0))

    while Q.size() > 0:
        c = Q.dequeue()
        if str(c.point) not in results:
            results[str(c.point)] = c.cost
        for p in neighboring_points(c.point, walls):
            if p not in visited:
                n = Node(p, distance(c.point, p) + c.cost)
                Q.enqueue(n)
                visited.add(p)

    return results

################################################################################
############################ CUSTOM HELPER METHODS #############################
################################################################################

def calculate_visited_point_density(point, history):

    """
    Returns how many states we have been to around 2 of the given point.
    """
    search_radius = 1

    (x, y) = point
    num_visited_neighbors = 0
    for i in range(-search_radius,search_radius+1):
        for j in range(-search_radius,search_radius+1):
            if i != 0 and j != 0:
                if str((x + i, y + j)) in history:
                    num_visited_neighbors += 1
    return num_visited_neighbors



def distance(p1, p2):
    """
    Returns the euclidian distance between the two given points.
    """
    (x1,y1) = p1
    (x2,y2) = p2
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

def moving_to_this_state_straight_up_kill_you(state, walls):
    """
    Returns true if moving to this state is certain death. False otherwise.
    """

    wall_hit = False
    for child in control_error_states(state):
        loc = state[0]
        new_loc = child[0]
        if rt.crash((loc,new_loc),walls):
            wall_hit = True
    return wall_hit

def hit_or_near_goal(state, fline):
    """
    Returns true if the given state is touching or one away from the goal. False otherwise.
    """
    ((x,y),(u,v)) = state
    ((fx1,fy1), (fx2,fy2)) = fline
    fx_start = min(fx1, fx2)
    fx_stop  = max(fx1, fx2)
    fy_start = min(fy1, fy2)
    fy_stop  = max(fy1, fy2)

    if x >= fx_start - 1 and x <= fx_stop + 1:
        if y >= fy_start and y <= fy_stop:
            return True

    if y >= fy_start - 1 and y <= fy_stop + 1:
        if x >= fx_start and x <= fx_stop:
            return True

    return False

def state_in_bounds(state, grid):
    """
    Returns true if the given state is in the bounds defined by the grid.
    False otherwise.
    """
    ((x,y),(u,v)) = state
    max = len(grid) - 1
    return x >= 0 and y >= 0 and x <= max and y <= max


def control_error_states(state):
    """
    Returns a set of possible states that can occur after the
    control error is applied to the given state.
    """
    ((x,y),(u,v)) = state
    states = set()
    for error_x in range(-1,2):     # ie. -1, 0, 1
        for error_y in range(-1,2): # ie. -1, 0, 1
            states.add(((x + error_x, y + error_y),(u + error_x, v + error_y)))

    return states

def neighboring_points(point, walls):
    """
    Returns the direct neighbors for the given point.
    Trims neighboring points that are touching walls.
    Used by the BFS to determine what states to search.
    """
    neighbors = set()
    (x, y) = point
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            neighboring_point = (x + dx, y + dy)
            is_valid = True
            for wall in walls:
                if rt.intersect((point,neighboring_point), wall):
                    is_valid = False
            if is_valid:
                neighbors.add(neighboring_point)
    return neighbors


def my_states(state, walls, fline):
    """
    Returns a set of possible states that we can move to starting at the
    given state.

    We trim states that will crash, states that will lead to certain death,
    and states that have locations we have been to before.
    """

    data2_file = open('data2.txt', 'r')
    history = json.load(data2_file)
    data2_file.close()

    ((x,y),(u,v)) = state
    states = set()
    for delta_x in [-2,-1,0,1,2]:
        for delta_y in [-2,-1,0,1,2]:
            move = ((x, y), (x + delta_x, y + delta_y))
            new_state = ((x + delta_x, y + delta_y),(delta_x, delta_y))
            if not rt.crash(move, walls) and str(new_state[0]) not in history:
                if hit_or_near_goal(state, fline):
                    # If that state is a goal we MUST be able to go to it (if we don't crash trying to do so).
                    states.add(new_state)
                elif not moving_to_this_state_straight_up_kill_you(new_state, walls):
                    # But, if it is NOT a goal then we can only go to it if we don't crash along the way and if moving to it won't kill us.
                    states.add(new_state)

    return states


################################################################################
############################ CUSTOM HELPER CLASSES #############################
################################################################################
class Node:
    """
    A node used in our BFS.
    """
    def __init__(self, point, cost):
        self.point = point
        self.cost = cost

class Stack:
    """
    A standard Stack.
    """
    def __init__(self):
     self.items = []

    def isEmpty(self):
     return self.items == []

    def push(self, item):
     self.items.append(item)

    def pop(self):
     return self.items.pop()

    def peek(self):
     return self.items[len(self.items)-1]

    def size(self):
     return len(self.items)

class Queue:
    """
    A standard Queue.
    """
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def enqueue(self, item):
        self.items.insert(0,item)

    def dequeue(self):
        return self.items.pop()

    def size(self):
        return len(self.items)


################################################################################
########################## MEMBERS FOR INITIALIZATION ##########################
################################################################################

def initialize(state,fline,walls):
    """
    Call edist_grid to initialize the grid for h_walldist, then write the data, in
    json format, to the file "data.txt" so it won't be lost when the process exits
    """

    bfs_results = bfs_heuristic(walls, fline)
    data_file = open('data.txt', 'w')
    json.dump(bfs_results,data_file)
    data_file.close()

    data2_file = open('data2.txt', 'w')
    history = dict()
    json.dump(history, data2_file)
    data2_file.close()
