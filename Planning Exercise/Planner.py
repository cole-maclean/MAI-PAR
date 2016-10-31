class Planner():
    def __init__(self,initial_state,office_count):
        self.office_count = office_count
        self.state = initial_state

    def possible_actions(self):
        self.action_list = []
        robo_office = self.state['robot-location']
        for office in range(0,self.office_count + 1):
            if office != robo_office:
                self.action_list.append({'move':[robo_office,office]})
        if self.state['robot-free'] and robo_office in self.state['machines'].keys():
            self.action_list.append({'make':[robo_office,self.state['machines'][robo_office]]})
        if robo_office in self.state['petitions'].keys():
            if self.state['robot-loaded'] == self.state['petitions'][robo_office]:
                self.action_list.append({'serve':[robo_office,self.state['robot-loaded']]})
        return self.action_list

    def make(self,office,cups):
        pass

    def move(self,office1,office2):
        pass

    def serve(self,office,cups):
        pass

    def check_goal_state(self):
        pass

