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
    # print("123")
    # print(problem.getSuccessors(problem.getStartState()))
    # print("123")
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

    start = problem.getStartState()
    # print("Start:", start)
    closed = util.Counter()     #Create closed (full info)
    fringe = fringeStack()      #Initial start node, fringe: [Node(state, action, cost, parent), ...]
    result = []

    fringe.push(Node(start, 0, 0, start))      
    while True:
        if (fringe.isEmpty()):
            print("Not Solution")
            return []           #Not solution

        node = fringe.pop()
        #Get current state of node
        currentState = node.getState()               
        #Add node into closed
        closed[hash(currentState)] = [node.getAction(), node.getCost(), node.getParent()]                  

        if (problem.isGoalState(currentState)):     # Check goal state

            nextState = currentState                    
            while(nextState != start):                  # If nextState == startState -> stop
                result.append(closed[hash(nextState)][0])     # append action with result from goal to start
                nextState = closed[hash(nextState)][2]        # assign parrent for childState   

            result.reverse()
            return result           #Return Solution (array of e,w,s,n)

        
        for x in problem.getSuccessors(currentState):   #Expand node from current node
            childState = x[0]
            if (closed[hash(childState)] == 0): #and not fringe.isExist(childState)): DFS don't care frontier
                child = Node(x[0], x[1], x[2], currentState)
                fringe.push(child)                      #Push child into frontier 
   
    # util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    start = problem.getStartState()
    # print("Start:", start)
    closed = util.Counter()     #Create closed (full info)
    fringe = fringeQueue()      #Initial start node, fringe: [Node(state, action, cost, parent), ...]
    result = []

    fringe.push(Node(start, 0, 0, start))      
    while True:
        if (fringe.isEmpty()):
            print("Not Solution")
            return []           #Not solution

        node = fringe.pop()
        #Get current state of node
        currentState = node.getState()             
        #Add node into closed
        closed[hash(currentState)] = [node.getAction(), node.getCost(), node.getParent()]                
        if (problem.isGoalState(currentState)):     # Check goal state

            nextState = currentState                    
            while(nextState != start):                  # If nextState == startState -> stop
                result.append(closed[hash(nextState)][0])     # append action with result from goal to start
                nextState = closed[hash(nextState)][2]        # assign parrent for childState   

            result.reverse()
            return result           #Return Solution (array of e,w,s,n)

        
        for x in problem.getSuccessors(currentState):   #Expand node from current node
            childState = x[0]
            if (closed[hash(childState)] == 0 and not fringe.isExist(childState)):
                child = Node(x[0], x[1], x[2], currentState)
                fringe.push(child)                      #Push child into frontier 
   
    # util.raiseNotDefined()
    

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    start = problem.getStartState()
    # print("Start:", start)
    closed = util.Counter()     #Create closed (full info)
    fringe = fringeHeap()       #Initial start node, fringe: [Node(state, action, cost, parent), ...]
    result = []

    fringe.push(Node(start, 0, 0, start), 999)      
    while True:
        if (fringe.isEmpty()):
            print("Not Solution")
            return []           #Not solution

        node = fringe.pop()
        #Get current state of node
        currentState = node.getState()               
        #Add node into closed
        closed[hash(currentState)] = [node.getAction(), node.getCost(), node.getParent()]                  

        if (problem.isGoalState(currentState)):     # Check goal state

            nextState = currentState                    
            while(nextState != start):                  # If nextState == startState -> stop
                result.append(closed[hash(nextState)][0])     # append action with result from goal to start
                nextState = closed[hash(nextState)][2]        # assign parrent for childState   

            result.reverse()
            return result           #Return Solution (array of e,w,s,n)

        
        for x in problem.getSuccessors(currentState):   #Expand node from current node
            childState = x[0]
            child = Node(x[0], x[1], x[2] + node.getCost(), currentState)
            if (closed[hash(childState)] == 0 and not fringe.isExist(childState)):
                #Push child into frontier 
                fringe.push(child, child.getCost())                     
            elif (fringe.isExist(childState) and fringe.getCost(childState) > child.getCost()):
                #Replace node in frontier
                fringe.replace(child)                                       

    # util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***" 

    start = problem.getStartState()
    # print("Start:", start)
    closed = util.Counter()     #Create closed (full info)
    fringe = fringeHeap()       #Initial start node, fringe: [Node(state, action, cost, parent), ...]
    result = []

    fringe.push(Node(start, 0, 0, start), heuristic(start, problem))      
    while True:
        if (fringe.isEmpty()):
            print("Not Solution")
            return []           #Not solution

        node = fringe.pop()

        #Get current state of node
        currentState = node.getState()        
        #Add node into closed
        closed[hash(currentState)] = [node.getAction(), node.getCost(), node.getParent()]                  

        if (problem.isGoalState(currentState)):     # Check goal state

            nextState = currentState                    
            while(nextState != start):                  # If nextState == startState -> stop
                result.append(closed[hash(nextState)][0])     # append action with result from goal to start
                nextState = closed[hash(nextState)][2]        # assign parrent for childState   

            result.reverse()
            return result           #Return Solution (array of e,w,s,n)

        
        for x in problem.getSuccessors(currentState):   #Expand node from current node
            childState = x[0]
            child = Node(x[0], x[1], x[2] + node.getCost(), currentState)
            if (closed[hash(childState)] == 0 and not fringe.isExist(childState)):
                #Push child into frontier 
                fringe.push(child, child.getCost() + heuristic(childState, problem))                     
            elif (fringe.isExist(childState) and fringe.getCost(childState) > child.getCost()):
                #Replace node in frontier
                fringe.replace(child, heuristic(childState, problem)) 

    # util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch


################################

class Node:
    def __init__(self, state, action, cost, parent):
        self.state = state
        self.action = action 
        self.cost = cost
        self.parent = parent

    def getParent(self):
        return self.parent

    def getAction(self):
        return self.action

    def getState(self):
        return self.state

    def getCost(self):
        return self.cost

class fringeQueue:
    def __init__(self):
        self.list = []
        self.hash = util.Counter()          

    def push(self, node):                   #node: Node(state, action, cost, parent)
        self.list.insert(0, node.getState())
        self.hash[hash(node.getState())] = [node.getAction(), node.getCost(), node.getParent()]     #hash[x] = action, cost, parent

    def pop(self):                          #return: Node(state, action, cost, parent)
        x = self.list.pop()
        acp = self.hash[hash(x)]
        return Node(x, acp[0], acp[1], acp[2])

    def isEmpty(self):
        return len(self.list) == 0

    def isExist(self, state):
        return self.hash[hash(state)] != 0

    def getNode(self, state):
        acp = self.hash[hash(state)]
        return Node(state, acp[0], acp[1], acp[2])

    def length(self):
        return len(self.list)

class fringeStack:
    def __init__(self):
        self.list = []
        self.hash = util.Counter()          

    def push(self, node):                   #node: Node(state, action, cost, parent)
        self.list.append(node.getState())
        self.hash[hash(node.getState())] = [node.getAction(), node.getCost(), node.getParent()]     #hash[x] = action, cost, parent

    def pop(self):                          #return: Node(state, action, cost, parent)
        x = self.list.pop()
        acp = self.hash[hash(x)]
        return Node(x, acp[0], acp[1], acp[2])

    def isEmpty(self):
        return len(self.list) == 0

    def isExist(self, state):
        return self.hash[hash(state)] != 0

    def getNode(self, state):
        acp = self.hash[hash(state)]
        return Node(state, acp[0], acp[1], acp[2])

    def length(self):
        return len(self.list)

class fringeHeap:
    def __init__(self):
        self.heap = util.PriorityQueue()
        self.hash = util.Counter()          

    def push(self, node, priority):                   #node: Node(state, action, cost, parent)
        self.heap.push(node.getState(), priority)
        self.hash[hash(node.getState())] = [node.getAction(), node.getCost(), node.getParent()]     #hash[x] = action, cost, parent

    def pop(self):                          #return: Node(state, action, cost, parent)
        state = self.heap.pop()
        acp = self.hash[hash(state)]
        return Node(state, acp[0], acp[1], acp[2])

    def isEmpty(self):
        return self.heap.isEmpty()

    def isExist(self, state):
        return self.hash[hash(state)] != 0

    def getNode(self, state):
        acp = self.hash[hash(state)]
        return Node(state, acp[0], acp[1], acp[2])

    def length(self):
        return len(self.heap.count)

    def getCost(self, state):
        return self.hash[hash(state)][1]  

    def replace(self, node, heuristic=0):
        self.hash[hash(node.getState())] = [node.getAction(), node.getCost(), node.getParent()]
        self.heap.update(node.getState(), node.getCost() + heuristic)



