import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        '''TASK 1 CONSTS
        [
                       {'agent':0,'loc':[(1,5)],'timestep':10},
                           {'agent':0,'loc':[(1,5)],'timestep':4},
                            {'agent':1,'loc':[(1,2),(1,3)],'timestep':1},
                       ]
        '''
        constraints = []
        '''Add code to prioritized.py that adds all necessary vertex constraints. You need two loops, namely
        one to iterate over the path of the current agent and one to add vertex constraints for all future
        agents (since constraints apply only to the specified agent). 
        '''
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            # for t in range(len(path)-1):
            #    for j in range(i+1, self.num_of_agents):
            #        constraints.append({'loc': [path[t],path[t+1]], 'timestep': t, 'agent': j})
            # constraints.append({'loc': [path[len(path)]], 'timestep': len(path), 'agent': i})
            result.append(path)
            for next_agent in range(self.num_of_agents):
                  for time in range(len(path)):
                    if next_agent != i:
                        constraints.append({'agent' : next_agent, 'loc' : [path[time]], 'timestep' : time})
                        if time > 0:
                            constraints.append({'agent' : next_agent, 'loc' : [path[time],path[time-1]], 'timestep' : time})

            
            
        
        
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
