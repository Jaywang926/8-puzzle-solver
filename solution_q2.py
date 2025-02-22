from collections import deque
import heapq

file = open("input.txt", "r").read()
start_state = tuple(file.replace('\n', '').split(','))

goal_state = ('1', '2', '3', '8', '_', '4', '7', '6', '5')

# goal matrix: [['1', '2', '3'],
#               ['8', '_', '4'], 
#               ['7', '6', '5']]

class Node:
    def __init__(self, value):
        self.value = tuple(value)
        self.path = ""
        self.num = 0
        
    def get_num(self):
        return self.num
    
    def __str__(self):
        return ".".join(self.value)
    
    __repr__ = __str__


def get_all_children(node):
    index = node.value.index('_')
    children = []

    #blank goes left
    if index not in {0, 3, 6}:
        new_state = list(node.value)
        new_state[index], new_state[index - 1] = new_state[index - 1], new_state[index] 
        new_node = Node(new_state)
        new_node.path = node.path + new_state[index] + 'R,'
        new_node.num = node.value[index - 1]
        children.append(new_node)

    #blank goes right
    if index not in {2, 5, 8}:
        new_state = list(node.value)
        new_state[index], new_state[index + 1] = new_state[index + 1], new_state[index]
        new_node = Node(new_state)
        new_node.path = node.path + new_state[index] + 'L,'
        new_node.num = node.value[index + 1]
        children.append(new_node)
    
    #blank goes up
    if index not in {0, 1, 2}: 
        new_state = list(node.value)
        new_state[index], new_state[index - 3] = new_state[index - 3], new_state[index] 
        new_node = Node(new_state)
        new_node.path = node.path + new_state[index] + 'D,'
        new_node.num = node.value[index - 3]
        children.append(new_node) 

    #blank goes down
    if index not in {6, 7, 8}:
        new_state = list(node.value)
        new_state[index], new_state[index + 3] = new_state[index + 3], new_state[index] 
        new_node = Node(new_state)
        new_node.path = node.path + new_state[index] + 'U,'
        new_node.num = node.value[index + 3]
        children.append(new_node)

    return children
   

def calc_manhattan_dist(num, node, goal):
    # find location of some number
    num_index = node.value.index(num)
    num_row = num_index // 3
    num_col = num_index % 3

    # find location of same number in the goal state
    goal_index = goal.index(num)
    goal_row = goal_index // 3
    goal_col = goal_index % 3

    horiz_dist = abs(num_col-goal_col)
    vert_dist = abs(num_row-goal_row)

    return horiz_dist + vert_dist


def calc_eucl_dist(num, node, goal):
    # find location of some number
    num_index = node.value.index(num)
    num_row = num_index // 3
    num_col = num_index % 3

    # find location of same number in the goal state
    goal_index = goal.index(num)
    goal_row = goal_index // 3
    goal_col = goal_index % 3

    horiz_dist = abs(num_col-goal_col)
    vert_dist = abs(num_row-goal_row)

    return (horiz_dist**2 + vert_dist**2)**0.5
   

def DFS_solve(node, goal, vis, depth, exps):
    vis.add(node.value) 

    if node.value == goal: 
        #print('DFS node expansions:', exps)
        return node.path[:-1]
    
    if node is None or depth == 100:
        return
    
    for child in get_all_children(node): 

        if child.value not in vis: 
            vis.add(child.value)
            exps += 1
            result = DFS_solve(child, goal, vis, depth + 1, exps)

            #check if a path was found
            if result:
                return result 
    return

node_exps = 0
start = Node(start_state)
visited = set()
depth = 0
solution = DFS_solve(start, goal_state, visited, depth, node_exps)
if solution:
    print('The solution to Q1.1a is:\n' + solution)

else: print('The solution to Q1.1a does not exist.')


def BFS_solve(start, goal):
    queue = deque()
    queue.append(start)
    visited = set()
    node_exps = 0

    while queue:
        curr_node = queue.popleft()
        visited.add(curr_node.value)

        if curr_node.value == goal:
            #print('BFS node expansions:', node_exps)
            return curr_node.path[:-1]
        
        for child in get_all_children(curr_node):
            if child.value not in visited:
                node_exps += 1
                queue.append(child)
    return

start = Node(start_state)
solution = BFS_solve(start, goal_state)
if solution:
    print('The solution to Q1.1b is:\n' + solution)

else: print('The solution to Q1.1b does not exist.')


def UCS_solve(start, goal):   
    heap = []
    visited = set()
    id = 0
    heapq.heappush(heap, (0, id, start))
    node_exps = 0

    while heap:
        cost, node_id, curr_node = heapq.heappop(heap)
        visited.add(curr_node.value)

        if curr_node.value == goal:
            #print('UCS node expansions:', node_exps)
            return curr_node.path[:-1]
        
        children = get_all_children(curr_node)
        for child in children:
            
            if  child.value not in visited:
                id += 1
                node_exps += 1
                heapq.heappush(heap, (cost + 1, id, child))
    return

start = Node(start_state)
solution = UCS_solve(start, goal_state)
if solution:
    print('The solution to Q1.1c is:\n' + solution)

else: print('The solution to Q1.1c does not exist.')


def A_manhattan_solve(start, goal):
    heap = []
    visited = set()
    id = 0
    heapq.heappush(heap, (0, id, 0, start))
    node_exps = 0

    while heap:
        expected_cost, node_id, curr_cost, curr_node = heapq.heappop(heap)
        visited.add(curr_node.value)

        if curr_node.value == goal:
            #print('A* with manhattan node expansions:', node_exps)
            return curr_node.path[:-1]
        
        children = get_all_children(curr_node)
        for child in children:
            dist = calc_manhattan_dist(child.get_num(), curr_node, goal)

            if child.value not in visited:
                id += 1
                node_exps += 1
                heapq.heappush(heap, (curr_cost + dist, id, curr_cost + 1, child))
    return

start = Node(start_state)
solution = A_manhattan_solve(start, goal_state)
if solution:
    print('The solution to Q1.1d is:\n' + solution)

else: print('The solution to Q1.1d does not exist.')


def A_eucl_solve(start, goal):
    heap = []
    visited = set()
    id = 0
    heapq.heappush(heap, (0, id, 0, start))
    node_exps = 0

    while heap:
        expected_cost, node_id, curr_cost, curr_node = heapq.heappop(heap)
        visited.add(curr_node.value)

        if curr_node.value == goal:
            #print('A* with euclidean node expansions:', node_exps)
            return curr_node.path[:-1]
        
        children = get_all_children(curr_node)
        for child in children:
            dist = calc_eucl_dist(child.get_num(), curr_node, goal)

            if child.value not in visited:
                id += 1
                node_exps += 1
                heapq.heappush(heap, (curr_cost + dist, id, curr_cost + 1, child))
    return

start = Node(start_state)
solution = A_eucl_solve(start, goal_state)
if solution:
    print('The solution to Q1.1e is:\n' + solution)

else: print('The solution to Q1.1e does not exist.')