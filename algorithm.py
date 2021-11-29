from collections import deque
from math import sqrt
import queue
import time
import psutil


# Parent Map Implementation
def bfs(initial_state):
    start_time = time.time()
    frontier = [initial_state]
    explore = set()
    parent_map = {}
    frontier_state = {initial_state: True}
    depth_map = {initial_state: 0}
    max_depth = 0
    while len(frontier) != 0:
        state = frontier.pop(0)
        explore.add(state)
        frontier_state.pop(state)

        if state == "012345678":
            return find_path(parent_map, initial_state, len(explore), max_depth, time.time() - start_time)

        else:

            for child in generate_children(state):
                if child not in frontier_state and child not in explore:
                    parent_map[child] = state
                    frontier_state[child] = True
                    frontier.append(child)
                    depth_map[child] = depth_map[state] + 1
                    if depth_map[child] > max_depth:
                        max_depth = depth_map[child]

    return []


def dfs(initial_state):
    start_time = time.time()
    frontier = deque()
    frontier.append(initial_state)
    frontier_state = {initial_state: True}
    explore = set()
    parent_map = {}
    depth_map = {initial_state: 0}
    max_depth = 0
    while len(frontier) != 0:
        state = frontier.pop()  # Stack behaviour
        frontier_state.pop(state)
        explore.add(state)

        if state == "012345678":
            return find_path(parent_map, initial_state, len(explore), max_depth, time.time() - start_time)
        else:
            for child in reversed(generate_children(state)):
                if child not in frontier_state and child not in explore:
                    parent_map[child] = state
                    frontier_state[child] = True
                    frontier.append(child)
                    depth_map[child] = depth_map[state] + 1
                    if depth_map[child] > max_depth:
                        max_depth = depth_map[child]

    return []


def is_solvable(state):
    inv_count = 0
    empty_value = 0
    for i in range(0, 9):
        for j in range(i + 1, 9):
            if int(state[j]) != empty_value and int(state[i]) != empty_value and int(state[i]) > int(state[j]):
                inv_count += 1
    if inv_count % 2 == 0:
        return True
    return False


def swap(s, i, j):
    lst = list(s)
    lst[i], lst[j] = lst[j], lst[i]
    return ''.join(lst)


def generate_children(state):
    state = str(state)
    children = []

    # Create children Nodes

    # Get index of empty tile
    index = state.index("0")

    # Check Move up
    if index > 2:
        children.append(swap(state, index, index - 3))

    # Check Move down
    if index < 6:
        children.append((swap(state, index, index + 3)))

    # Check Left move
    if index % 3 > 0:
        children.append(swap(state, index, index - 1))

    # Check Right move
    if index % 3 < 2:
        children.append(swap(state, index, index + 1))

    return children


# printing the state in 3x3 board
def print_path(game, cost, nodes_explored, depth, time_taken):
    game = game[::-1]  # reverse the path
    for state in game:
        for i in range(9):
            if i == 2 or i == 5 or i == 8:
                print(state[i] + " ")
            else:
                print(state[i], end=" ")
        print("---------------------------------------------")
    print("Cost = " + str(cost))
    print("Depth = " + str(depth))
    print("Nodes Explored = " + str(nodes_explored))
    print("Running Time = " + str(time_taken))
    # print("Cost = "+str(cost) + "  Nodes Explored = "+str(nodes_explored))


# backtracking the path from the goal state to the initial state
def find_path(parent_map, initial_state, nodes_explored, depth, time_taken):
    current_state = "012345678"  # to get the parent of the goal state
    path = [current_state]
    cost = 0
    while current_state != initial_state:
        path.append(parent_map[current_state])
        current_state = parent_map.get(current_state)
        cost += 1
    # return path
    print_path(path, cost, nodes_explored, depth, time_taken)


def A_star(initial_state, heuristic):
    start_time = time.time()

    frontier = queue.PriorityQueue()
    # frontier[initial_state] = heuristic(initial_state)
    frontier.put([heuristic(initial_state), initial_state])
    explored = set()
    parent_map = {}
    depth_map = {initial_state: 0}
    max_depth = 0
    while frontier.qsize() > 0:
        # prev_cost = frontier.peekitem()[1] - heuristic(frontier.peekitem()[0])
        state1 = frontier.get()
        prev_cost = state1[0]
        state = state1[1]
        explored.add(state)

        if state == "012345678":
            return find_path(parent_map, initial_state, len(explored), max_depth, time.time() - start_time)

        for child in generate_children(state):
            if child not in frontier.queue and child not in explored:
                parent_map[child] = state
                # frontier[child] = heuristic(child) + prev_cost + 1
                frontier.put([heuristic(child) + prev_cost + 1, child])
                depth_map[child] = depth_map[state] + 1
                if depth_map[child] > max_depth:
                    max_depth = depth_map[child]

    return []


"""
Heuristic functions
=> Manhattan Distance
=> Euclides Distance
"""


def manhattan_distance(state, goal="012345678", col=3):
    h = 0
    for c in state:
        index = state.find(c)
        target = goal.find(c)
        x_index = index % col
        y_index = index // col
        x_target = target % col
        y_target = target // col
        h = h + abs(x_index - x_target) + (y_index - y_target)
    return h


def euclides_distance(state, goal="012345678", col=3):
    h = 0
    for c in state:
        index = state.find(c)
        target = goal.find(c)
        x_index = index % col
        y_index = index // col
        x_target = target % col
        y_target = target // col
        h = h + sqrt((x_index - x_target) ** 2 + (y_index - y_target) ** 2)
    return h


def solver(initial_state, algorithm, heuristic=None):
    solution = None
    if algorithm == 1:
        # solution = BFS_node(initial_state)
        bfs(initial_state)
    elif algorithm == 2:
        # solution = DFS_node(initial_state)
        solution = dfs(initial_state)
    elif algorithm == 3:
        if heuristic == 0:
            A_star(initial_state, manhattan_distance)
        else:
            A_star(initial_state, euclides_distance)
    end_mem = psutil.virtual_memory().used
    return solution, end_mem
