##Make stack more obvious, don't let serve from mahcine of 3 to 2.

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
    def __init__(self,initial_state):
        self.office_count = 36 #numer of offices for grid creation
        self.state = initial_state
        self.operator_list = ['move','make','serve']
        self.stack_history = [{'action':'initialize','state':copy.deepcopy(self.state),'stack':[]}] #store initial state in plan. deep copy needed to store unmutated state at each iteration
        self.plan = ['initialize']
        self.goal_state = [('served',office) for office in self.state['petitions']] #the goal state is to have served all of the offices in the petition list
        self.stack = []
        for goal in self.goal_state: #initialize goal stack with goal state conditions
            self.stack.append(goal)

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

    def get_preconds(self,operator):
        #check the preconditions for each operator
        op = operator[0]
        params = operator[1]
        if op == 'make':
            return [('robot-free',True),('robot-location',params[0])]
        elif op == 'move':
            return [('robot-location',params[0])]
        elif op == 'serve':
            return [('robot-location',params[0]),('robot-loaded',params[1])]

    def get_additions(self,operator):
        #check additions for each operator
        op = operator[0]
        params = operator[1]
        if op == 'make':
            return [('robot-loaded',params[1])]
        elif op == 'move':
            return [('robot-location',params[1])]
        elif op == 'serve':
            return [('served',params[0]),('robot-free',True)]

    def apply_operator(self,operator):
        #apply the selected operator to the current state
        act = operator[0]
        params = operator[1]
        if act == 'make':
            self.make(*params)
        elif act == 'move':
            self.move(*params)
        else:
            self.serve(*params)

    def order_service(self):
        #heuristic for ordering service goals in the stack based on closest machine to the robot that can serve one of the petitions
        services = [service for service in self.stack if service[0] == 'served']
        if services:
            min_dist = 36
            for srv in services:
                for office,capacity in self.state['machines'].items():
                    if capacity == self.state['petitions'][srv[1]]:
                        if manhattan_distance(office,self.state['robot-location']) <= min_dist:
                            closest_service = srv
                            min_dist = manhattan_distance(office,self.state['robot-location'])
            self.stack[self.stack.index(closest_service)] = self.stack[len(services)-1]
            self.stack[len(services)-1] = closest_service
        return self.stack

    def order_make(self,cups):
        #heuristic for ordering the make operators in the stack to select the make operation closest to the robot in the current state
        closest_make =[]
        min_dist = 36
        for office,capacity in self.state['machines'].items():
            if capacity == cups:
                if manhattan_distance(office,self.state['robot-location']) < min_dist:
                    closest_make = ['make',[office,cups]]
                    min_dist = manhattan_distance(office,self.state['robot-location'])
        return closest_make

    def update_stack(self):
        #update the current stack of goals
        self.order_service() #apply order service heuristic
        stack_goal = self.stack[-1] #select the bottom of the stack goal condition 
        if stack_goal[0] in self.operator_list: #check if selected goal is an operator
            preconditioned = True 
            preconds = self.get_preconds(stack_goal) #get all the preconditions of the selected operator
            for pre in preconds: #check if each precondition is satisfied in current state
                if self.state[pre[0]] != pre[1] not in self.stack: #if the precondition is not satisfied, and it is not alreay in the stack, add it to the stack
                    self.stack.append(pre)
                    preconditioned = False #operation is not precontioned, so can not apply it to current state
            if preconditioned == True: #if selected operator satisfies all preconditions, add it to end of plan and apply it to the current state
                self.plan.append(stack_goal) #add operator to plan
                self.stack_history.append({'state':copy.deepcopy(self.state),'action':stack_goal,'stack':copy.deepcopy(self.stack)}) #update stack history
                self.apply_operator(stack_goal) #transition to new state by applying operator to current state
                indx = self.stack.index(stack_goal)#pop operator from the stack of goals
                del self.stack[indx]
            else:
                self.stack_history.append({'state':copy.deepcopy(self.state),'action':'','stack':copy.deepcopy(self.stack)})#update stack history
        else:
            for operator in self.operator_list: #if goal is not an operator, search through all the opertors to find one that has in its add list the selected goal
                if operator == 'make':
                    action = self.order_make(stack_goal[1])
                    if action:      
                        if stack_goal in self.get_additions(action):#if selected goal is in the make operators add list, add the make operator to the stack and delete the goal condition from the stack
                            self.stack.append(action)
                            indx = self.stack.index(stack_goal) #remove goal condition
                            del self.stack[indx]
                elif operator == 'move':
                    action = ['move',[self.state['robot-location'],stack_goal[1]]]      
                    if stack_goal in self.get_additions(action): #if selected goal is in the move operators add list, add the make operator to the stack and delete the goal condition from the stack
                        self.stack.append(action)
                        indx = self.stack.index(stack_goal)
                        del self.stack[indx]
                elif operator == 'serve':
                    try:
                        action = ['serve',[stack_goal[1],self.state['petitions'][stack_goal[1]]]]
                        if stack_goal in self.get_additions(action): #if selected goal is in the serve operators add list, add the make operator to the stack and delete the goal condition from the stack
                            self.stack.append(action)
                            indx = self.stack.index(stack_goal)
                            del self.stack[indx]
                    except KeyError:
                        pass
            self.stack_history.append({'state':copy.deepcopy(self.state),'action':'','stack':copy.deepcopy(self.stack)}) #update stack history
        return self.stack

    def build_plan(self):
        #build plan by execution perform_step while stack is not empty
        while self.stack:
            self.update_stack()
        self.plan.append('finished')
        self.stack_history.append({'state':copy.deepcopy(self.state),'action':'','stack':copy.deepcopy(self.stack)})
        return self.stack_history

