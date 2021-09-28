# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    
    # Empty stack is created to keep the track of the nodes which are not visited yet.
    # Stack is used because DFS uses stack data structure.
    opened = util.Stack()
    
    # Keeps the track of visited nodes.
    closed = []
    
    # Keeps the track of all the nodes to reach to the goal node .
    path = []
    
    # Cost to reach to the goal node.
    pathCost = 0
    
    # Start node from which game starts.
    startNode = problem.getStartState()
    
    # the opened list consists of the state, path to that node,
    # and cost to reach to that node.
    # Initially start node is pushed in the opened list.
    opened.push((startNode, [], pathCost))
    
    # Loop until opened list becomes empty.
    while not opened.isEmpty():
        # pop() operation is performed on opened list.
        # It returns current state, path to current node, and cost.
        state, path, cost = opened.pop()
        
        # If current state is not in closed list, i.e. the node is not visited yet,
        # then it is pushed in the closed list
        if state not in closed:
            closed.append(state)
            
            # Checks if the current state is goal state. If so then it returns the path to that node.
            if problem.isGoalState(state):
                return path
            # getSuccessor() method returns the list of successors of the current node.
            # it consists of the list of successors, paths to that successor and cost.
            successors = problem.getSuccessors(state)
            
            # For each successor node, it checks whether the successor node is already visited or not.
            # If not, then it pushes them into the open list.
            for successor in successors:
                if successor[0] not in closed:
                    opened.push((successor[0], path + [(successor[1])], successor[2]))
    
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    
    # Empty queue is created to keep the track of the nodes which are not visited yet.
    # Queue is used because BFS uses queue data structure.
    opened = util.Queue()
    
    # Keeps the track of visited nodes.
    closed = []
    
    # Keeps the track of all the nodes to reach to the goal node .
    path = []
    
    # Cost to reach to the goal node.
    pathCost = 0
    
    # Start node from which game starts.
    startNode = problem.getStartState()
    
    # the opened list consists of the state, path to that node,
    # and cost to reach to that node.
    # Initially start node is pushed in the opened list.
    opened.push((startNode, [], pathCost))
    
    # Loop until opened list becomes empty.
    while not opened.isEmpty():
        # pop() operation is performed on opened list.
        # It returns current state, path to current node, and cost.
        state, path, cost = opened.pop()
    
        # If current state is not in closed list, i.e. the node is not visited yet,
        # then it is pushed in the closed list
        if state not in closed:
            closed.append(state)
            
        # Checks if the current state is goal state. If so then it returns the path to that node.
        if problem.isGoalState(state):
            return path
        
        # getSuccessor() method returns the list of successors of the current node.
        # it consists of the list of successors, paths to that successor and cost.
        successors = problem.getSuccessors(state)
        
        # For each successor node, it checks whether the successor node is already visited or not.
        # If not, then it checks whether it is already available in opened list, i.e. unexplored.
        # If not then pushes them into the open list.
        for successor in successors:
            if successor[0] not in closed: 
                if successor[0] not in (unExploredState[0] for unExploredState in opened.list):
                    opened.push((successor[0], path + [(successor[1])], successor[2]))
    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    
    # Empty priority queue is created to keep the track of the nodes which are not visited yet.
    # Priority Queue is used because uniform cost search algorithm uses priority queue data structure.
    # It prioritizes the successors having lower cost.
    opened = util.PriorityQueue()
    
    # Keeps the track of visited nodes.
    closed = []
    
    # Keeps the track of all the nodes to reach to the goal node .
    path = []
    
    # Initially priority is set to zero.
    priority = 0
    
    # Cost to reach to the goal node.
    pathCost = 0
    
    # Start node from which game starts.
    startNode = problem.getStartState()
    
    # the opened list consists of the state, path to that node,
    # and cost to reach to that node.
    # Initially start node is pushed in the opened list.
    opened.push((startNode, [], pathCost),priority)
    
    # Loop until opened list becomes empty.
    while not opened.isEmpty():
        # pop() operation is performed on opened list.
        # It returns current state, path to current node, and cost.
        state, path, cost = opened.pop()
        
        # If current state is not in closed list, i.e. the node is not visited yet,
        # then it is pushed in the closed list
        if state not in closed:
            closed.append(state)
            
            # Checks if the current state is goal state. If so then it returns the path to that node.
            if problem.isGoalState(state):
                return path
            
            # getSuccessor() method returns the list of successors of the current node.
            # it consists of the list of successors, paths to that successor and cost.
            successors = problem.getSuccessors(state)
            
            # For each successor node, it checks whether the successor node is already visited or not.
            # If not, then it pushes them into the open list.
            for successor in successors:
                if successor[0] not in closed:
                    opened.push((successor[0], path + [(successor[1])], cost + successor[2]), cost + successor[2])
    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
