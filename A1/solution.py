#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems
import heapq

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    manhattan_distance = 0
    for box in state.boxes:
      #for each box, calculate manhattan distance for each available storage spot and find the closest one
      min_heap = []
      for storage in state.storage:
        distance = abs(box[0]-storage[0]) + abs(box[1]-storage[1])
        heapq.heappush(min_heap, distance)
      manhattan_distance += min_heap[0]
      
    return manhattan_distance


#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def box_in_corner(box, state):
  #check if each box, that is not on a storage, is against a corner
  x,y = box

  if (x,y) not in state.storage:
    if x == 0 and (y == 0 or y == state.height-1):
      return True
    if x == state.width-1 and (y == 0 or y == state.height-1):
      return True

  return False

# def goal_pull(state):
#   # For every goal:
#   #  1. Delete all boxes from the board
#   #  2. Place a box at the goal square
#   #  3. PULL the box from the goal square to every possible square and mark all reached squares as visited
#   distance_to_goal = {}
#   possible_box_positions= set()
#   obstacles = state.boxes.union(state.obstacles)

#   queue = deque()
#   directions = [[0,1],[1,0],[-1,0],[0,-1]]

#   for storage in state.storage:
#     distance_to_goal[(storage, storage)] = 0
#     queue.append(storage)

#     while queue:
#       position = queue.popleft()

#       for direction in directions:
#         box_position = (position[0] + direction[0], position[1]+direction[1])
#         robot_position = (position[0] + 2*direction[0], position[1]+ 2*direction[1])

#         if (storage, box_position) not in distance_to_goal:
#           if not wall_at_position(box_position, state) and not wall_at_position(robot_position, state) and robot_position not in obstacles:
#             distance_to_goal[(storage, box_position)] = distance_to_goal[(storage, position)]+ 1
#             queue.append(box_position)
#             possible_box_positions.add(box_position)
  
#   return distance_to_goal, possible_box_positions

# def wall_at_position(position, state):
#   x,y = position
#   if y == state.height or y == -1 or x == state.width or x == -1:
#     return True


def box_against_obstacle(box, state):
  x,y = box

  if (x,y) in state.storage:
    return False

  obstacles = state.boxes.union(state.obstacles)
  storage_x = [storage[0] for storage in state.storage]
  storage_y = [storage[1] for storage in state.storage]

  left = (x-1, y)
  right = (x+1, y)
  up = (x, y+1)
  down = (x, y-1)
  

  #against edges or if there is no goal along edge
  #top edge
  if y == state.height-1 and ((left in obstacles or right in obstacles) or (y not in storage_y)):
    return True
  #left edge
  if x == 0 and ((up in obstacles or down in obstacles) or ((x not in storage_x))):
    return True
  #right edge
  if x == state.width-1 and ((up in obstacles or down in obstacles) or ((x not in storage_x))):
    return True
  #bottom edge
  if y == 0 and ((left in obstacles or right in obstacles) or (y not in storage_y)):
    return True
  

  if up in obstacles and (left in obstacles or right in obstacles):
    up_l = (up[0]-1,up[1])
    up_r = (up[0]+1,up[1])
    if up_l in obstacles or up_r in obstacles:
      return True

  if down in obstacles and (left in obstacles or right in obstacles):
    down_l = (down[0]-1,down[1])
    down_r = (down[0]+1,down[1])
    if down_l in obstacles or down_r in obstacles:
      return True
  
  #check for freeze deadlocks
  #top right corner
  if (state.width-1 , state.height-1) not in state.storage:
    if (x,y) == (state.width-2,state.height-1) and (state.width-1,state.height-2) in obstacles:
      return True
    if (x,y) == (state.width-1,state.height-2) and (state.width-2,state.height-1) in obstacles:
      return True
  
  #top left corner
  if (0 , state.height-1) not in state.storage:
    if (x,y) == (1 , state.height-1) and (0,state.height-2) in obstacles:
      return True
    if (x,y) == (0,state.height-2) and (1 , state.height-1) in obstacles:
      return True
  
  #bot left corner
  if (0 , 0) not in state.storage:
    if (x,y) == (0,1) and (1,0) in obstacles:
      return True
    if (x,y) == (1,0) and (0,1) in obstacles:
      return True
  
  #bot right corner
  if (state.width-1, 0) not in state.storage:
    if (x,y) == (state.width-1, 1)  and (state.width-2, 0) in obstacles:
      return True
    if (x,y) == (state.width-2, 0) and (state.width-1, 1)  in obstacles:
      return True

  return False


def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    
    for box in state.boxes:
      if box_in_corner(box, state) or box_against_obstacle(box, state):
        return float("inf") 

    result_heuristic = 0
    available_storage = set(state.storage)
    for box in state.boxes:
      #for each box, calculate manhattan distance for each available storage spot and find the closest one
      min_heap = []
      for storage in available_storage:
        distance = abs(box[0]-storage[0]) + abs(box[1]-storage[1])
        heapq.heappush(min_heap, (distance, storage))
      result_heuristic += min_heap[0][0]
      available_storage.remove(min_heap[0][1]) #remove from available storage points
      
    return result_heuristic

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the standard form of weighted A* (i.e. g + w*h)

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + (weight * sN.hval)

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime weighted astar algorithm'''
  wrapped_fval_function = (lambda sN : fval_function(sN, weight))

  # #instantiate search engine
  search_engine = SearchEngine(strategy="custom", cc_level="full")
  search_engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)

  end_time = os.times()[0] + timebound
  cost_bound = (float('inf'),float('inf'),float('inf'))
  result = False

  while os.times()[0]<end_time:
    result_state = search_engine.search(end_time - os.times()[0], cost_bound)[0]

    if result_state:
      cost_bound = (float("inf"), float("inf"), result_state.gval + heur_fn(result_state))
      # decrease weight after each iteration
      weight = weight * 0.5
      result = result_state
    else:
      break

  return result

 

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime greedy best-first search'''

  #instantiate search engine
  search_engine = SearchEngine(strategy="best_first", cc_level="full")
  search_engine.init_search(initial_state, sokoban_goal_state, heur_fn)

  end_time = os.times()[0] + timebound
  cost_bound = (float('inf'),float('inf'),float('inf'))
  result = False

  while os.times()[0]<end_time:
    result_state = search_engine.search(end_time - os.times()[0], cost_bound)[0]
    if result_state:
      cost_bound = (result_state.gval-1,float('inf'), float('inf'))
      result = result_state
    else:
      break

  return result

