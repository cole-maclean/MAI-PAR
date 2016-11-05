
<h1>Coffee Delivery Robot Linear Planner</h1>
<b>MAI-PAR Planning Exercise</b>  
<b>Cole MacLean and Lucky Chinedu</b>  
<b>November 6, 2016</b>  

<h2>Introduction</h2>
A discrete state planning exercise is presented to utilize and test the linear planning with a stack of goals ([PRISM](http://www.cs.cmu.edu/~mmv/planning/readings/strips.pdf)) algorithm for agent planning.  

A mobile robotic agent capable of making and serving coffee to offices within a 2D office building is tasked with efficiently serving the requested coffee petitions in the least amount of steps to reduce office worker disturbances. The office building is modelled as a 6x6 square grid of 36 offices, and the steps between 2 offices is defined as their manhattan distance. The agent is fed information about coffee machine and petition capacities and office locations, and is required to build a daily service plan to efficiently fulfill the petitions given the new office configuration.

The agent is a linear planner with 3 operators: Move, Make and Serve. The preconditions, additions and deletions of each operator in the planning algorithm are listed below.

Make(o,n): the robot makes n cups of coffee in the machine located at office o
<ul>
 <li>Preconditions: robot-location(o), robot-free, machine(o,n)</li>
 <li>Add: robot-loaded(n)</li>
 <li>Delete: robot-free</li>
</ul>

Move(o1,o2): the robot moves from o1 to o2
<ul>
    <li>Preconditions: robot-location(o1), steps(x)</li>
    <li>Add: robot-location(o2), steps(x+distance(o1,o2))</li>
    <li>Delete: robot-location(o1), steps(x)</li>
</ul>

Serve(o,n): the robot delivers n cups of coffee to office o
<ul>
<li>Preconditions: robot-location(o), robot-loaded(n), petition(o,n)</li>
<li>Add: served(o), robot-free</li>
<li>Delete: petition(o,n), robot-loaded(n)</li>
</ul>

A linear planner using these operators and some simple search heuristics has been implemented and can be tested [here](http://ponderinghydrogen.pythonanywhere.com/)
<img src="example_state.PNG">

<h2>Algorithm Implementation</h2>
The linear planner is implementated as a class object with 3 main methods that perform: Action search space traversal, Action filtering hueristics and Action execution.  
<b>Initialization</b>  
The Planner class is initializated with the initial state of the form   
                              {'robot-location': o, 'steps': 0,  
                              'petitions': {office:petition_count},  
                              'robot-free': True, 'robot-loaded': 0,  
                              'machines': {office:capacity},  
                              'served': []}


```python
import Planner
```


```python
my_planner = Planner.Planner({'robot-location': 3, 'steps': 0,
                              'petitions': {2: 1, 10: 3, 11: 1, 12: 2, 24: 1},
                              'robot-free': True, 'robot-loaded': 0,
                              'machines': {3: 3, 7: 1, 20: 2, 22: 1,30:2},
                              'served': []},36)
```

<b>Search Space Traversal</b>  
The first method, possible_actions, checks the current state compared to operator preconditions and builds a list of all legal actions.


```python
acts = my_planner.possible_actions()
acts
```




    [['move', [3, 0]],
     ['move', [3, 1]],
     ['move', [3, 2]],
     ['move', [3, 4]],
     ['move', [3, 5]],
     ['move', [3, 6]],
     ['move', [3, 7]],
     ['move', [3, 8]],
     ['move', [3, 9]],
     ['move', [3, 10]],
     ['move', [3, 11]],
     ['move', [3, 12]],
     ['move', [3, 13]],
     ['move', [3, 14]],
     ['move', [3, 15]],
     ['move', [3, 16]],
     ['move', [3, 17]],
     ['move', [3, 18]],
     ['move', [3, 19]],
     ['move', [3, 20]],
     ['move', [3, 21]],
     ['move', [3, 22]],
     ['move', [3, 23]],
     ['move', [3, 24]],
     ['move', [3, 25]],
     ['move', [3, 26]],
     ['move', [3, 27]],
     ['move', [3, 28]],
     ['move', [3, 29]],
     ['move', [3, 30]],
     ['move', [3, 31]],
     ['move', [3, 32]],
     ['move', [3, 33]],
     ['move', [3, 34]],
     ['move', [3, 35]],
     ['make', [3, 1]],
     ['make', [3, 2]],
     ['make', [3, 3]]]



<b>Search Space Filtering Heuristics</b>  
The method, select_action, uses 4 simple rules to select the optimal action from the list of all possible actions.
#1.) If a serve action exists in the action_list, then perform the serve action
#2.) If robot is loaded with a petitioned amount of cups, perform move action to the minimum manhatten distance to the petitioning offices
#3.) If a make action is in the list, and #ofcups equals an existing petitioned #ofcups, then perform the make action that can serve the closest petition
#4.) If no other actions than move to a machine exist, move to the closest machine capable of making #ofcups > or = an existing petitioned #ofcups


```python
my_planner.select_action(acts)
```




    ['make', [3, 1]]



<b>Action Execution</b>  
The method, perform_step, executes the selected action and stores the resulting new state in the planner.plan property. A helper function, build_plan, iteratively performs the perform_step method until the goal_state is satisfied, and returns the full plan with intermediate states and actions for the agent.


```python
my_planner.build_plan()
```




    [{'action': 'initialize',
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {2: 1, 10: 3, 11: 1, 12: 2, 24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 3,
       'served': [],
       'steps': 0}},
     {'action': ['make', [3, 1]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {2: 1, 10: 3, 11: 1, 12: 2, 24: 1},
       'robot-free': False,
       'robot-loaded': 1,
       'robot-location': 3,
       'served': [],
       'steps': 0}},
     {'action': ['move', [3, 2]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {2: 1, 10: 3, 11: 1, 12: 2, 24: 1},
       'robot-free': False,
       'robot-loaded': 1,
       'robot-location': 2,
       'served': [],
       'steps': 1}},
     {'action': ['serve', [2, 1]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {10: 3, 11: 1, 12: 2, 24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 2,
       'served': [2],
       'steps': 1}},
     {'action': ['move', [2, 3]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {10: 3, 11: 1, 12: 2, 24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 3,
       'served': [2],
       'steps': 2}},
     {'action': ['make', [3, 3]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {10: 3, 11: 1, 12: 2, 24: 1},
       'robot-free': False,
       'robot-loaded': 3,
       'robot-location': 3,
       'served': [2],
       'steps': 2}},
     {'action': ['move', [3, 10]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {10: 3, 11: 1, 12: 2, 24: 1},
       'robot-free': False,
       'robot-loaded': 3,
       'robot-location': 10,
       'served': [2],
       'steps': 4}},
     {'action': ['serve', [10, 3]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {11: 1, 12: 2, 24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 10,
       'served': [2, 10],
       'steps': 4}},
     {'action': ['move', [10, 3]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {11: 1, 12: 2, 24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 3,
       'served': [2, 10],
       'steps': 6}},
     {'action': ['make', [3, 1]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {11: 1, 12: 2, 24: 1},
       'robot-free': False,
       'robot-loaded': 1,
       'robot-location': 3,
       'served': [2, 10],
       'steps': 6}},
     {'action': ['move', [3, 11]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {11: 1, 12: 2, 24: 1},
       'robot-free': False,
       'robot-loaded': 1,
       'robot-location': 11,
       'served': [2, 10],
       'steps': 9}},
     {'action': ['serve', [11, 1]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {12: 2, 24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 11,
       'served': [2, 10, 11],
       'steps': 9}},
     {'action': ['move', [11, 3]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {12: 2, 24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 3,
       'served': [2, 10, 11],
       'steps': 12}},
     {'action': ['make', [3, 2]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {12: 2, 24: 1},
       'robot-free': False,
       'robot-loaded': 2,
       'robot-location': 3,
       'served': [2, 10, 11],
       'steps': 12}},
     {'action': ['move', [3, 12]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {12: 2, 24: 1},
       'robot-free': False,
       'robot-loaded': 2,
       'robot-location': 12,
       'served': [2, 10, 11],
       'steps': 17}},
     {'action': ['serve', [12, 2]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 12,
       'served': [2, 10, 11, 12],
       'steps': 17}},
     {'action': ['move', [12, 7]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {24: 1},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 7,
       'served': [2, 10, 11, 12],
       'steps': 19}},
     {'action': ['make', [7, 1]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {24: 1},
       'robot-free': False,
       'robot-loaded': 1,
       'robot-location': 7,
       'served': [2, 10, 11, 12],
       'steps': 19}},
     {'action': ['move', [7, 24]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {24: 1},
       'robot-free': False,
       'robot-loaded': 1,
       'robot-location': 24,
       'served': [2, 10, 11, 12],
       'steps': 23}},
     {'action': ['serve', [24, 1]],
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 24,
       'served': [2, 10, 11, 12, 24],
       'steps': 23}},
     {'action': 'finished',
      'state': {'machines': {3: 3, 7: 1, 20: 2, 22: 1, 30: 2},
       'petitions': {},
       'robot-free': True,
       'robot-loaded': 0,
       'robot-location': 24,
       'served': [2, 10, 11, 12, 24],
       'steps': 23}}]



<h2>Final Results</h2>  
Using a simple linear planner with stack of goals and simple search space heuristics, an efficient agent for coffee service planning can be implemented. Through many different office configurations, the implemented agent performs logically and efficiently in collecting and serving petitions. 

<h2>Limitations</h2>
The described linear planner implementation does not gaurentee maximally optimal service plans. Although the 4 rules for search space filtering capture nearly all situations for optimal action selection, there does exist situations where the agent will perform sub-optimally. These cases arise when a coffee machine that can serve a petition is closer to that petiton, to another capable machine closer to the agent, causing the agent to unnecessarily. An example configuration of this is presented here.
<img src="limit_example1.PNG">

In this example, the optimal route is for the agent to use the 2 cup capacity machine to serve the single cup petition, and carry-on towards the 3 cup capacity machine to serve the 2 cup petition. The above implementation incorrectly utilize the 2 cup capacity machine to serve both petitions, due to search hueristic #4 move to closest machine capable of serving an existing petition. 

<img src="limit_example2.PNG">

Future work to improve the above implementation and correct this suboptimal behaviour would be to implement full search tree traversal and selection of the globally optimal plan. However, the advantages of the above implementation is its superior ability to scale to larger input configuration, compared to the computationally expensive method of a full search tree traversal implementation. 

<h2>Execution Instructions</h2>
A graphical interface for the described agent can be found [here](http://ponderinghydrogen.pythonanywhere.com/) To initialize the agent, fill in the desired coffee machine capacities and petitions in the correct offices and input the initial office of the robot in the 'robot_cell' input box. For example, if office 16 has a machine with capacity of 3 cups, and office 23 has a petition for 2 cups, input 3 in the 'machines' input box of office 16, and 2 in the 'petitions' input box of office 23. If the agent starts in office 2, enter 2 in the 'robot_cell' input box. Once configured, select the 'Submit' button.  
You will be redirected to a graphical simulation of the agents plan, that can be stepped through using the 'Plan Step' slider to visualize plan execution.