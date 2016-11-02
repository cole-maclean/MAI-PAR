import math
import random
import copy
def office_cords(office,cols,rows):
    if office ==0:
        return 1,1
    else:
        return office % cols, math.ceil(office/rows)

def manhattan_distance(office1,office2):
    off1_x, off1_y = office_cords(office1,6,6)
    off2_x, off2_y = office_cords(office2,6,6)
    return abs(off1_x - off2_x) + abs(off1_x - off2_x)

class Planner():
    def __init__(self,initial_state,office_count):
        self.office_count = office_count
        self.state = initial_state
        self.plan = []
        self.goal_state = False

    def possible_actions(self):
        self.action_list = []
        robo_office = self.state['robot-location']
        for office in range(0,self.office_count):
            if office != robo_office:
                self.action_list.append(['move',[robo_office,office]])
        if self.state['robot-free'] and robo_office in self.state['machines'].keys():
            self.action_list.append(['make',[robo_office,self.state['machines'][robo_office]]])
        if robo_office in self.state['petitions'].keys():
            if self.state['robot-loaded'] == self.state['petitions'][robo_office]:
                self.action_list.append(['serve',[robo_office,self.state['robot-loaded']]])
        return self.action_list

    def select_action(self,action_list):
        serve_action = [act for act in action_list if act[0] == 'serve']
        if serve_action:
            return serve_action[0]

        serve_move_action = [act for act in action_list if act[0] == 'move' and self.state['petitions'].get(act[1][1],None) == self.state['robot-loaded']]
        if serve_move_action:
            return serve_move_action[0]

        make_action = [act for act in action_list if act[0] == 'make' and act[1][1] in self.state['petitions'].values()]
        if make_action:
            return make_action[0]

        make_move_action = [act for act in action_list if act[0] == 'move' and self.state['machines'].get(act[1][1],None) in self.state['petitions'].values()]

        return make_move_action[0]

    def make(self,office,cups):
        self.state['robot-loaded'] = cups
        self.state['robot-free'] = False
        return self.state

    def move(self,office1,office2):
        self.state['robot-location'] = office2
        self.state['steps'] = self.state['steps'] + manhattan_distance(office1,office2)
        return self.state

    def serve(self,office,cups):
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
        actions = self.possible_actions()
        selected_action = self.select_action(actions)
        self.plan.append({'state':copy.deepcopy(self.state),'action':selected_action})
        act = selected_action[0]
        params = selected_action[1]
        if act == 'make':
            self.make(*params)
        elif act == 'move':
            self.move(*params)
        else:
            self.serve(*params)
        self.check_goal_state()
        return self.plan

    def build_plan(self):
        while self.goal_state == False and self.state['steps'] < 1000:
            self.perform_step()
        self.plan.append({'state':self.state,'action':{}})
        return self.plan