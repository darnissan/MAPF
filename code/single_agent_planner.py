import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0),(0,0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost,}
            if (child_loc in closed_list):
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    '''constrain structure
    {'agent': 2,
'loc': [(3,4)],
'timestep': 5}
    '''
    constrain_table = dict()
    for constraint in constraints:
        if constraint['agent'] == agent:
            if constraint['timestep'] in constrain_table:
                constrain_table[constraint['timestep']].append(constraint['loc'])
            else:
                constrain_table[constraint['timestep']] = constraint['loc']
    
    
    return constrain_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    '''1.3 Adding Edge Constraints
We now consider (negative) edge constraints, that prohibit a given agent from moving from a given
cell to another given cell at a given time step.
The following code creates a (negative) edge constraint that prohibits agent 2 from moving from cell
(1, 1) to cell (1, 2) from time step 4 to time step 5:
{'agent': 2,
'loc': [(1,1), (1,2)],
'timestep': 5}
Implement constraint handling for edge constraints in the function is_constrained.
You can test your code by adding a constraint in prioritized.py that prohibits agent 1 from
moving from its start cell (1, 2) to the neighboring cell (1, 3) from time step 0 to time step 1
    ''' 
    #if len(constraint_table) != 0 and next_time==3: 
        #breakpoint()
    if next_time in constraint_table:
        if next_loc in constraint_table[next_time] :
            return True
        if  (len(constraint_table[next_time])==len(next_loc) and next_loc == constraint_table[next_time]):
            return True
        if [curr_loc,next_loc] in constraint_table[next_time] or [curr_loc,next_loc] == [constraint_table[next_time]] :
          
            return True
    return False



def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
  
    
    
   
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    print("now with agent" + str(agent))
    constrain_table=build_constraint_table(constraints, agent)
    print("the constraint table for agent" +str(agent) +" is  \n"+ str(constrain_table))
    print("the len of the constraint table is " + str(len(constrain_table)))
    intKey = []
    if len(constrain_table) == 0:
        earliest_goal_timestep = 0
    else:
        for constraint in constrain_table.items():
          
            if [constraint[1]] == [goal_loc]:
                
                intKey.append(constraint[0])
        earliest_goal_timestep = max(intKey)+1 if len(intKey) != 0 else 0
    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None,'timestep':0}
    push_node(open_list, root)
    closed_list[(root['loc'],root['timestep'])] = root
    
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            return get_path(curr)
        for dir in range(5):
           
            child_loc = move(curr['loc'], dir)
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            if is_constrained(curr['loc'], child['loc'], child['timestep'], constrain_table):
                
                continue
            if (child['loc'],child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'],child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'],child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'],child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
