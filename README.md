### EXP NO: 04

### DATE: 17/05/2022


# <p align = "center"> A* Path Finding Algorithm for 2D Grid World </p>

## AIM

To develop a code to find the route from the source to the destination point using A* algorithm for 2D grid world.

## THEORY
A-star (also referred to as A*) is one of the most successful search algorithms to find the shortest path between nodes or graphs. It is an informed search algorithm, as it uses information about path cost and also uses heuristics to find the solution.A* Search algorithms, unlike other traversal techniques, it has “brains”. What it means is that it is really a smart algorithm which separates it from the other conventional algorithms. 

## DESIGN STEPS

### STEP 1:
Import the neccessary packages and define some suitable classes and functions.

### STEP 2:
Build a 2D grid world with initial state(1,1) and goal state(5,10). 

### STEP 3:
Mention the obstacles and define any distance method.

### STEP 4:
Pass the initial state,final state,obstacles and distance method and receive the directions as output.


## <br><br>Draw the 2D 
![Screenshot (181)](https://user-images.githubusercontent.com/75243072/168870942-304c6e60-e391-4535-ba90-5964b3ada3e8.png)

## PROGRAM
```
Developed By
Student name : Kumaran.B
Reg.no : 212220230026
```
```python
%matplotlib inline
import matplotlib.pyplot as plt
import random
import math
import sys
from collections import defaultdict, deque, Counter
from itertools import combinations
import heapq
class Problem(object):
    """The abstract class for a formal problem. A new domain subclasses this,
    overriding `actions` and `results`, and perhaps other methods.
    The default heuristic is 0 and the default action cost is 1 for all states.
    When yiou create an instance of a subclass, specify `initial`, and `goal` states 
    (or give an `is_goal` method) and perhaps other keyword args for the subclass."""

    def __init__(self, initial=None, goal=None, **kwds): 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        
        raise NotImplementedError
    def result(self, state, action): 
        raise NotImplementedError
    def is_goal(self, state):        
        return state == self.goal
    def action_cost(self, s, a, s1): 
        return 1
    
    def __str__(self):
        return '{0}({1}, {2})'.format(
            type(self).__name__, self.initial, self.goal)
class Node:
    "A Node in a search tree."
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __str__(self): 
        return '<{0}>'.format(self.state)
    def __len__(self): 
        return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): 
        return self.path_cost < other.path_cost
failure = Node('failure', path_cost=math.inf) # Indicates an algorithm couldn't find a solution.
cutoff  = Node('cutoff',  path_cost=math.inf) # Indicates iterative deepening search was cut off.
def expand(problem, node):
    "Expand a node, generating the children nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]
class PriorityQueue:
    """A queue in which the item with minimum f(item) is always popped first."""

    def __init__(self, items=(), key=lambda x: x): 
        self.key = key
        self.items = [] # a heap of (score, item) pairs
        for item in items:
            self.add(item)
         
    def add(self, item):
        """Add item to the queuez."""
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)

    def pop(self):
        """Pop and return the item with min f(item) value."""
        return heapq.heappop(self.items)[1]
    
    def top(self): return self.items[0][1]

    def __len__(self): return len(self.items)
def best_first_search(problem, f):
    "Search nodes with minimum f(node) value first."
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
    return failure

def g(n): 
    return n.path_cost
class GridProblem(Problem):
    """Finding a path on a 2D grid with obstacles. Obstacles are (x, y) cells."""

    def __init__(self, initial=(15, 30), goal=(130, 30), obstacles=(), **kwds):
        Problem.__init__(self, initial=initial, goal=goal, 
                         obstacles=set(obstacles) - {initial, goal}, **kwds)

    directions = [(-1, -1), (0, -1), (1, -1),
                  (-1, 0),           (1,  0),
                  (-1, +1), (0, +1), (1, +1)]
    
    def action_cost(self, s, action, s1): 
        return straight_line_distance(s, s1)
    
    def h(self, node): 
        return straight_line_distance(node.state, self.goal)
                  
    def result(self, state, action): 
        "Both states and actions are represented by (x, y) pairs."
        return action if action not in self.obstacles else state
    
    def actions(self, state):
        """You can move one cell in any of `directions` to a non-obstacle cell."""
        x,y = state
        return {(x+dx,y+dy) for(dx,dy) in self.directions}-self.obstacles
   
def straight_line_distance(A, B):
    "Straight-line distance between two points."
    return sum(abs(a-b)**2 for (a,b) in zip(A,B))**0.5

def g(n): 
    return n.path_cost
def astar_search(problem, h=None):
    """Search nodes with minimum f(n) = g(n) + h(n)."""
    h = h or problem.h
    return best_first_search(problem, f=lambda n: g(n) + h(n))
obstacles={(1,2),(2,3),(5,1),(4,4),(4,5),(2,6),(2,7),(3,7),(4,9),(5,7),(1,10)}    
grid1 = GridProblem(initial=(1,1), goal =(5,10) ,obstacles=obstacles)
s2 = astar_search(grid1)
a=path_states(s2)
print(a)
```

<br><br><br>
## <br><br><br>OUTPUT:
![Screenshot (182)](https://user-images.githubusercontent.com/75243072/168871824-5853460e-e0e2-4d97-bf29-840268214bc8.png)

## Solution Justification:
A Star Search algorithm is the best algorithm using nowadays to find the shortest distance to reach the goal state.But,it is not true in all the cases.There must be a possibility to give a long distance route(less possible).However, it is good technique to find short distance to reach goal state.

## <br><br><br>RESULT:
Hence, A* Algorithm was implemented to find the route from the source to the destination point in a 2D gird World.
