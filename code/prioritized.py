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
        
        '''
        Constraints by Task
        Task 1.2
        {'agent': 0, 'loc': [(1, 5)], 'timestep': 4}
        
        Task 1.3
        {'agent': 1, 'loc': [(1,2),(1, 3)], 'timestep': 1}
        
        Task 1.4
        {'agent': 0, 'loc': [(1, 5)], 'timestep': 10}
        
        Task 1.5
        {'agent': 1, 'loc': [(1, 3)], 'timestep': 2}
        {'agent': 1, 'loc': [(1, 2)], 'timestep': 2}
        {'agent': 1, 'loc': [(1, 4)], 'timestep': 2}
        '''
        
        
        
        
        
        constraints = []
       
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            #Task 2.1 - constraining the path of the next agent according to the current agent's path
            for next_agent in range(i+1,self.num_of_agents):
                for time in range(len(path)):
                        constraints.append({'agent' : next_agent, 'loc' : [path[time]], 'timestep' : time})
                        if time > 0:
                            #Task 2.2 - constraining the path of the next agent according to the current agent's path with edge constraints
                            constraints.append({'agent' : next_agent, 'loc' : [path[time],path[time-1]], 'timestep' : time})
            #Task 2.3 - blocking reached goals
                while True:
                    if path is None:
                       raise BaseException('No solutions')
                    next_agent_path=a_star(self.my_map, self.starts[next_agent], self.goals[next_agent], self.heuristics[next_agent], next_agent, constraints)
                    if next_agent_path is None:
                        raise BaseException('No solutions')
                    #Task 2.4 - limiting the length of the path
                    if len(next_agent_path)>=len(path) + len(self.my_map)*len(self.my_map[0]):
                        raise BaseException('No solutions')
                    blocked_goal=path[-1]
                    if blocked_goal in next_agent_path:
                        constraints.append({'agent' : next_agent, 'loc' : [blocked_goal], 'timestep' : next_agent_path.index(blocked_goal)}) # block the goal
                        constraints.append({'agent' : next_agent, 'loc' : [ path[-2]], 'timestep' : next_agent_path.index(blocked_goal)}) # block the location agent came from
                    else:
                        break
            
                        
            
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
           
            
            
            
        
        
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
