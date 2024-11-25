# CBCA-Python
This is a Python implementation of Consensus-Based Bundle Algorithm (CBCA).

You can see more details about CBCA from these papers.

* [Choi, H.-L., Brunet, L., and How, J. P., “Consensus-Based Decentralized Auctions for Robust Task Allocation,” IEEE Transactions on Robotics, vol. 25, Aug. 2009, pp. 912–926.](https://ieeexplore.ieee.org/abstract/document/5072249?casa_token=zYvs9usD3FYAAAAA:jz0SmSso6T5l107pHGJgIQhVNP3S4NEnnIPi6sRC--8aealzVFinApRitUzhISlprmsPjcr3)

* [Brunet, L., Choi, H.-L., and How, J. P., “Consensus-Based Auction Approaches for Decentralized Task Assignment,” AIAA Guidance, Navigation, and Control Conference (GNC), Honolulu, HI: 2008.](https://arc.aiaa.org/doi/abs/10.2514/6.2008-6839)

Require:
Python >= 3.7

This repo has been tested with:
* Python 3.9.1, macOS 11.2.1, numpy 1.20.1, matplotlib 3.3.4
* python 3.8.5, Ubuntu 20.04.2 LTS, numpy 1.20.1, matplotlib 3.3.4


Dependencies
============
For Python:
* [numpy](https://numpy.org/)
* [matplotlib](https://matplotlib.org/)

```
$ pip3 install numpy matplotlib
```


Usage
=====

The parameters for Tasks and Robots are written in a configuration json file.
* `ROBOT_TYPES`: robot type.
* `TASK_TYPES`: task type. The i-th task type is associated with the i-th robot type.
* `NOM_VELOCITY`: the average speed of robot [m/s].
* `TASK_VALUE`: the value/reward of task. With larger value, the task is more important than others.
* `START_TIME`: the starting timestamp of task [sec].
* `END_TIME`: the enging timestamp of task [sec].
* `DURATION`: the duration/time window of task [sec]. An robot needs to arrive at a task before the starting time and stays there until the ending time to be counted as complete a task.

An example `config_example_01.json`:
```json
{
    "ROBOT_TYPES": ["quad", "car"],
    "TASK_TYPES": ["track", "rescue"],

    "QUAD_DEFAULT": {
        "NOM_VELOCITY": 2
    },

    "CAR_DEFAULT": {
        "NOM_VELOCITY": 2
    },
    
    "TRACK_DEFAULT": {
        "TASK_VALUE": 100,
        "START_TIME": 0,
        "END_TIME": 150,
        "DURATION": 15
    },

    "RESCUE_DEFAULT": {
        "TASK_VALUE": 100,
        "START_TIME": 0,
        "END_TIME": 150,
        "DURATION": 5
    }
}
```

The algorithm's main function is `CBCA.solve()`. An example with task time window is shown below.
```python
#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import pathmagic
with pathmagic.context():
    from CBCA import CBCA
    from WorldInfo import WorldInfo
    import HelperLibrary as HelperLibrary


if __name__ == "__main__":
    # a json configuration file
    config_file_name = "config_example_01.json"
    # Read the configuration from the json file
    json_file = open(config_file_name)
    config_data = json.load(json_file)

    # create a world, each list is [min, max] coordinates for x,y,z axis
    WorldInfoTest = WorldInfo([-2.0, 2.5], [-1.5, 5.5], [0.0, 20.0])

    # create a list of Robot(s) and Task(s)
    num_robots = 5
    num_tasks = 10
    max_depth = num_tasks
    RobotList, TaskList = HelperLibrary.create_robots_and_tasks(num_robots, num_tasks, WorldInfoTest, config_data)

    # create a CBCA solver
    CBCA_solver = CBCA(config_data)

    # solve
    path_list, times_list = CBCA_solver.solve(RobotList, TaskList, WorldInfoTest, max_depth, time_window_flag=True)

    # path_list is a 2D list, i-th sub-list is the task exploration order of Robot-i.
    # e.g. path_list = [[0, 4, 3, 1], [2, 5]] means Robot-0 visits Task 0 -> 4 -> 3 -> 1, and Robot-1 visits Task 2 -> 5

    # times_list is a 2D list, i-th sub-list is the task beginning timestamp of Robot-i's tasks.
    # e.g. times_list = [[10.5, 20.3, 30.0, 48.0], [20.4, 59.5]] means Robot-0 arrives at Task-0 before time=10.5 second, and then conduct Task-0;
    # Robot-0 arrives at Task-4 before time=20.3 second, and then conduct Task-4, etc.

    # plot
    CBCA_solver.plot_assignment()
    plt.show()


```

An example without task time window is shown below.
```python
    # create a list of Robot(s) and Task(s)
    # create_robots_and_tasks_homogeneous() set Task.start_time, Task.duration, and Task.end_time as zero
    # when time_window_flag=False, CBCA doesn't consider Task.start_time, Task.duration, and Task.end_time
    RobotList, TaskList = HelperLibrary.create_robots_and_tasks_homogeneous(num_robots, num_tasks, WorldInfoTest, config_data)
    # create a CBCA solver
    CBCA_solver = CBCA(config_data)

    # solve, no time window
    path_list, _ = CBCA_solver.solve(RobotList, TaskList, WorldInfoTest, max_depth, time_window_flag=False)

```


Example
=======

A simple example with a task time window is [`run_CBCA_example_01.py`](/test/run_CBCA_example_01.py).
```
$ cd <MAIN_DIRECTORY>
$ python3 run_CBCA_example_01.py
```
The task assignment for each robot is stored as a 2D list `path_list` (the return variable of `CBCA.solve()`). The result visualization is shown below.
![A simple example with task time window a](/doc/1_a.png)
![A simple example with task time window b](/doc/1_b.png)


Another example with task time window (but the task duration is zero) is [`run_CBCA_example_02.py`](/test/run_CBCA_example_02.py).
```
$ cd <MAIN_DIRECTORY>
$ python3 run_CBCA_example_02.py
```
The task assignment for each robot is stored as a 2D list `path_list` (the return variable of `CBCA.solve()`). The result visualization is shown below.
![A simple example with task time window 2](/doc/2.png)


An example where tasks don't have time window is [`run_CBCA_example_03.py`](/test/run_CBCA_example_03.py).
```
$ cd <MAIN_DIRECTORY>
$ python3 run_CBCA_example_03.py
```
The result visualization is shown below.
![A simple example without task time window](/doc/3.png)


Not every task is assigned. What happened?
======================================================

There are some configuration settings in json files that can cause this problem.

If your tasks have time windows, i.e. `time_window_flag=True`, it is possible that no robot can execute a task due to the limitation of `START_TIME` or `END_TIME` for this task.
It can take a long time for every robot traveling to this task. Before an robot arrives at this location, this task has been expired.
So it is impossible for this task to be assigned to any robots.
A quick fix is to set a relatively large `END_TIME` or small `DURATION` if your world , i.e., `WorldInfo` is large.

More details in compute_bid() and scoring_compute_score() of [`CBCA.py`](/lib/CBCA.py)
