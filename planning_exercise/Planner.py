import math
import random
import copy
def office_cords(office,cols,rows):
    #convert office index to x,y coordinate
    return (office % cols), math.floor(office/rows)

def manhattan_distance(office1,office2):
    off1_x, off1_y = office_cords(office1,6,6)
    off2_x, off2_y = office_cords(office2,6,6)
    return abs(off1_x - off2_x) + abs(off1_y - off2_y)

class Planner():
    r"""The Planner class contains the methods used to build the coffee serving robots plan from initial state to goal state using a linear planner with stack of goals methodology (STRIPS)
    Reference: http://www.cs.cmu.edu/~mmv/planning/readings/strips.pdf
    """
    def __init__(self,initial_state,office_count):
        self.office_count = office_count
        self.state = initial_state
        self.plan = [{'action':'initialize','state':copy.deepcopy(self.state)}] #store initial state in plan. deep copy needed to store unmutated state at each iteration
        self.goal_state = False

    def possible_actions(self):
        r"""Return a list of allowable actions given the current state and satisfied pre-conditions. This method facilitates the pre-conditions check step for each operator in the STRIPS algorithm.
        Actions are stored as nested lists of the form [action,[*action_params]]
        """
        self.action_list = []
        robo_office = self.state['robot-location']
        #OPERATOR: MOVE    PRECONDITIONS: robot-location(o1)  DATA-STRUCTURE: ['move',[office1,office2]]
        #The preconditions for the move operator are always satisfied for movements from the robots current office to any other office.
        for office in range(0,self.office_count):
            #for all officies in the grid except the robots current office, append move(o1,o2) to action list
            if office != robo_office:
                self.action_list.append(['move',[robo_office,office]])

        #OPERATOR: MAKE    PRECONDITIONS:  robot-location(o), robot-free, machine(o,n)  DATA-STRUCTURE: ['make',[office,#ofcups]]
        #If the robot is free and in an office with a machine, append make(office,#ofcups) for all #ofcups from 1 to coffee machines capacity
        if self.state['robot-free'] and robo_office in self.state['machines'].keys():
            for cups in range(1,self.state['machines'][robo_office] + 1): 
                self.action_list.append(['make',[robo_office,cups]])


        #OPERATOR: SERVE    PRECONDITIONS:  robot-location(o), robot-loaded(n), petition(o,n)  DATA-STRUCTURE: ['serve',[office,#ofcups]]
        #If the robot is in an office with a petition, check if the robot has loaded the number of cups requested by the petition and add serve(office,#ofcups) to action list
        if robo_office in self.state['petitions'].keys():
            if self.state['robot-loaded'] == self.state['petitions'][robo_office]:
                self.action_list.append(['serve',[robo_office,self.state['robot-loaded']]])
        return self.action_list

    def select_action(self,action_list):
        r"""Select the optimal action for the robot to perform from the possible action list by searching using some basic logical hueristics. 
        """

        #1.) If a serve action exists in the action_list, then perform the serve action
        serve_action = [act for act in action_list if act[0] == 'serve']
        if serve_action:
            return serve_action[0]

        #2.) If robot is loaded with a petitioned amount of cups, perform move action to the minimum manhatten distance to the petitioning offices
        serve_move_action = [act for act in action_list if act[0] == 'move' and self.state['petitions'].get(act[1][1],None) == self.state['robot-loaded']]
        if serve_move_action:
            return min(serve_move_action, key=lambda x: manhattan_distance(x[1][0],x[1][1]))

        #3.) If a make action is in the list, and #ofcups equals an existing petitioned #ofcups, then perform the make action that can serve the closest petition
        make_action = [act for act in action_list if act[0] == 'make' and act[1][1] >= min(self.state['petitions'].values())]
        best_make_action_distance = 1000
        best_make_action = []
        #for all possible make actions, find the one that allows a serve_move_action with the least distance
        for possible_make_action in make_action:
            possible_petition_offices = [pet_office for pet_office,pet_cups in self.state['petitions'].items() if pet_cups == possible_make_action[1][1]]
            for petition_office in possible_petition_offices:
                distance = manhattan_distance(possible_make_action[1][0],petition_office)
                if distance < best_make_action_distance:
                    best_make_action_distance = distance
                    best_make_action = possible_make_action
        if best_make_action:
            return best_make_action

        #4.) If no other actions than move to a machine exist, move to the closest machine capable of making #ofcups > or = an existing petitioned #ofcups
        make_move_action = [act for act in action_list if act[0] == 'move' and self.state['machines'].get(act[1][1],0) >= min(self.state['petitions'].values())]
        if make_move_action:
            return min(make_move_action, key=lambda x: manhattan_distance(x[1][0],x[1][1]))

        return 'Out of Actions!'

    def make(self,office,cups):
        #OPERATOR: MAKE    ADD:  robot-loaded(n) DELETE: robot-free
        self.state['robot-loaded'] = cups
        self.state['robot-free'] = False
        return self.state

    def move(self,office1,office2):
        #OPERATOR: MOVE    ADD:  robot-location(office2) steps(x) DELETE: robot-location(office1)
        self.state['robot-location'] = office2
        self.state['steps'] = self.state['steps'] + manhattan_distance(office1,office2)
        return self.state

    def serve(self,office,cups):
        #OPERATOR: SERVE    ADD:  served(office) robot-free DELETE:  petition(office,cups) robot-loaded(cups)
        self.state['served'].append(office)
        self.state['robot-free'] = True
        self.state['petitions'].pop(office)
        self.state['robot-loaded'] = 0
        return self.state

    def check_goal_state(self):
        if self.state['petitions']:
            return False
        else:
            self.goal_state = True
            return True

    def perform_step(self):
        #perform single linear planner iteration by checking allowable actions and searching for optimal one to perform. Store state and selected action to plan list
        actions = self.possible_actions()
        selected_action = self.select_action(actions)
        act = selected_action[0]
        params = selected_action[1]
        if act == 'make':
            self.make(*params)
        elif act == 'move':
            self.move(*params)
        else:
            self.serve(*params)
        self.check_goal_state()
        self.plan.append({'state':copy.deepcopy(self.state),'action':selected_action}) #deep copy new state and action performed to obtain that state
        return self.plan

    def build_plan(self):
        #build plan by execution perform_step while goal state is not statisfied
        while self.goal_state == False:
            self.perform_step()
        self.plan.append({'state':self.state,'action':'finished'})
        return self.plan