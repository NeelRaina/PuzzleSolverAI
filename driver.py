import queue as Q

import time

import sys

import math

import heapq

import psutil


class PuzzleState(object):
    """docstring for PuzzleState"""

    def __init__(self, config, n, parent=None, action="Initial", cost=0):

        if n * n != len(config) or n < 2:
            raise Exception("the length of config is not correct!")

        self.n = n

        self.cost = cost

        self.parent = parent

        self.action = action

        self.dimension = n

        self.config = config

        self.children = []

        for i, item in enumerate(self.config):

            if item == 0:
                self.blank_row = i // self.n

                self.blank_col = i % self.n

                break

    def display(self):

        for i in range(self.n):

            line = []

            offset = i * self.n

            for j in range(self.n):
                line.append(self.config[offset + j])

            print(line)

    def move_left(self):

        if self.blank_col == 0:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index - 1

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Left", cost=self.cost + 1)

    def move_right(self):

        if self.blank_col == self.n - 1:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index + 1

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Right", cost=self.cost + 1)

    def move_up(self):

        if self.blank_row == 0:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index - self.n

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Up", cost=self.cost + 1)

    def move_down(self):

        if self.blank_row == self.n - 1:

            return None

        else:

            blank_index = self.blank_row * self.n + self.blank_col

            target = blank_index + self.n

            new_config = list(self.config)

            new_config[blank_index], new_config[target] = new_config[target], new_config[blank_index]

            return PuzzleState(tuple(new_config), self.n, parent=self, action="Down", cost=self.cost + 1)

    def expand(self):

        """expand the node"""

        # add child nodes in order of UDLR

        if len(self.children) == 0:

            up_child = self.move_up()

            if up_child is not None:
                self.children.append(up_child)

            down_child = self.move_down()

            if down_child is not None:
                self.children.append(down_child)

            left_child = self.move_left()

            if left_child is not None:
                self.children.append(left_child)

            right_child = self.move_right()

            if right_child is not None:
                self.children.append(right_child)

        return self.children


# Function that Writes to output.txt

### Students need to change the method to have the corresponding parameters

def listOfActions(state):
    actions = [state.action]
    former = state.parent

    for x in range(state.cost - 1):
        actions.append(former.action)
        former = former.parent

    return actions[::-1]


def writeOutput(pathToGoal, costOfPath, nodesExpanded, searchDepth, runningTime, maxRamUsage):
    output = open("output.txt", "w")
    output.write("path_to_goal: {}\n".format(pathToGoal))
    output.write("cost_of_path: {}\n".format(costOfPath))
    output.write("nodes_expanded: {}\n".format(nodesExpanded))
    output.write("search_depth: {}\n".format(costOfPath))
    output.write("max_search_depth: {}\n".format(searchDepth))
    output.write("running_time: {}\n".format(runningTime))
    output.write("max_ram_usage: {}".format(maxRamUsage))


### Student Code Goes here

def bfs_search(initial_state):
    timestamp1 = time.time()
    frontier = Q.Queue()
    listFrontier = set()
    frontier.put(initial_state)
    listFrontier.add(initial_state.config)
    visited = set()
    largest = 0
    while not frontier.empty():
        state = frontier.get()

        if test_goal(state):
            timestamp2 = time.time()
            statistic = psutil.Process().memory_info().rss
            runningTime = timestamp2 - timestamp1
            actions = listOfActions(state)
            costOfPath = state.cost
            return writeOutput(actions, costOfPath, len(visited), largest, runningTime, statistic)

        visited.add(state.config)
        listFrontier.remove(state.config)
        neighbhors = state.expand()
        for neighbhor in neighbhors:
            if neighbhor.config not in listFrontier and neighbhor.config not in visited:
                frontier.put(neighbhor)
                listFrontier.add(neighbhor.config)
                if neighbhor.cost > largest:
                    largest = neighbhor.cost

    """BFS search"""

    ### STUDENT CODE GOES HERE ###


def dfs_search(initial_state):
    print("test")
    timestamp1 = time.time()
    largest = 0
    frontier = [initial_state]
    listFrontier = set()
    listFrontier.add(initial_state.config)
    visited = set()

    while len(frontier) > 0:

        state = frontier.pop()
        listFrontier.remove(state.config)

        if test_goal(state):
            print("test2")
            timestamp2 = time.time()
            statistic = psutil.Process().memory_info().rss
            print(statistic)
            runningTime = timestamp2 - timestamp1
            print(runningTime)
            actions = listOfActions(state)
            costOfPath = state.cost
            print(costOfPath)
            return writeOutput(actions, costOfPath, len(visited), largest, runningTime, statistic)

        visited.add(state.config)

        neighbhors = state.expand()[::-1]

        for neighbhor in neighbhors:
            if neighbhor.config not in visited and neighbhor.config not in listFrontier:
                frontier.append(neighbhor)
                listFrontier.add(neighbhor.config)
                if neighbhor.cost > largest:
                    largest = neighbhor.cost
    """DFS search"""

    ### STUDENT CODE GOES HERE ###


def A_star_search(initial_state):
    timestamp1 = time.time()
    frontier = []
    count = 0
    heapq.heappush(frontier, (0, count, initial_state))

    visited = set()
    listFrontier = set()
    listFrontier.add(initial_state.config)
    largest = 0

    while len(frontier) > 0:
        state = heapq.heappop(frontier)
        listFrontier.remove(state[2].config)

        if test_goal(state[2]):
            timestamp2 = time.time()
            statistic = psutil.Process().memory_info().rss
            runningTime = timestamp2 - timestamp1
            actions = listOfActions(state[2])
            costOfPath = state[2].cost
            return writeOutput(actions, costOfPath, len(visited), largest, runningTime, statistic)
        visited.add(state[2].config)
        states = state[2].expand()
        for state in states:
            count += 1
            if state.config not in listFrontier and state.config not in visited:
                listFrontier.add(state.config)
                dist = calculate_total_cost(state)
                heapq.heappush(frontier, (dist, count, state))
                if state.cost > largest:
                    largest = state.cost

    """A * search"""

    ### STUDENT CODE GOES HERE ###


def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""
    return state.cost + calculate_manhattan_dist(state)
    ### STUDENT CODE GOES HERE ###


def calculate_manhattan_dist(state):
    """calculate the manhattan distance of a tile"""
    distance = 0
    board = state.config

    for tile in range(1, 9):
        distance += abs(tile % 3 - board.index(tile) % 3) + (abs(tile // 3 - board.index(tile) // 3))
    return distance

    ### STUDENT CODE GOES HERE ###


def test_goal(puzzle_state):
    """test the state is the goal state or not"""
    goal = (0, 1, 2, 3, 4, 5, 6, 7, 8)

    if goal == puzzle_state.config:
        return True
    return False
    ### STUDENT CODE GOES HERE ###


# Main Function that reads in Input and Runs corresponding Algorithm

def main():

    sm = sys.argv[1].lower()

    begin_state = sys.argv[2].split(",")

    begin_state = tuple(map(int, begin_state))

    size = int(math.sqrt(len(begin_state)))

    hard_state = PuzzleState(begin_state, size)

    if sm == "bfs":

        bfs_search(hard_state)

    elif sm == "dfs":

        dfs_search(hard_state)

    elif sm == "ast":

        A_star_search(hard_state)

    else:

        print("Enter valid command arguments !")

if __name__ == '__main__':
    main()