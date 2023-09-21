# Optimal Task Allocation for Heterogeneous Multi-robot Teams with Battery Constraints
## Overview
This repository deals with optimal task planning missions with heterogeneous teams of robots that will have to execute a
number of tasks that may have different specifications and types, while dealing with limited resources that can be
recovered by executing certain actions, i.e. the robots have limited battery power, but have the possibility to plan
recharges.

The objective is to find a mission plan, i.e. to assign all tasks to the agents needed to complete them successfully respecting the constraints imposed by each task, while minimizing the total execution time, i.e. the makespan.

The problem is formulated using Mixed Integer Linear Programming (MILP) in Matlab. To solve it, the `intlinprog()`
Matlab function is called. If you have Gurobi installed in Matlab, this function should be overwitten by the
`intlinprog()` Gurobi function, so the same code works using Matlab or Gurobi solvers.

## Problem description
The proposed problem constitutes a first step towards the formulation and analysis of a problem of complexity never before addressed in the state of the art.

The problem can be classified as ST-MR-TA according to the taxonomy described in [[1]](#1), i.e., robots can execute only one task at a time (ST), tasks may need more than one robot to execute (MR), and robots are assigned tasks to be executed according to a schedule. Can also be classified as Task Windows (TW) according to [[2]](#2). There's also another cathegory in that taxonomy, Synchronization and Precedence (SP). We do not hace precedence constraints, but we do have synchronizacion constraints when multiple robots has to execute a task together or do a relay due to battery constraints. Last, this problem can be classified also as CD according to [[3]](#3), i.e. the tasks has complex inter-task dependencies.

Robots are mainly characterized by their type. Tasks have a list of robot types that have hardware compatible with it. In this way, the formulation can restrict the assignment of tasks to task-compatible robots only, being totally agnostic to the actual hardware that constitutes each type of robot.

To understand the wide variety of tasks that are contemplated within the problem, we talk about two characteristics of the tasks: composability and number of robots. Depending on the composability, tasks can be non-decomposable, relayable or fragmentable. Non-composable tasks must be executed continuously by the same robots, relayable tasks must be executed continuously, but allow relays between several robots, and finally, fragmentable tasks can be executed in fragments that do not have to be coordinated between them and can be spread over the schedule of one or several robots. On the other hand, tasks can have a fixed number of robots, i.e. they need to be executed by exactly the specified number of robots, they can have a variable number of robots, meaning that they specify an ideal number of robots with which to be executed, but allow a different number of robots to be used, or they can specify no number of robots at all, and their duration will depend on the number of robots finally employed to execute them. Tasks can belong to any type that is a combination of these two characteristics.

In the [scenario](scripts/scenario.m) script you can see in more detail how agents and tasks are implemented within the formulation.

## Installation
There are no special requirements to be able to run the formulation Matlab script.
Just make sure you have correctly added this project's path to the matlab path, including subfolders:
```
addpath(genpath('<install_dir>/task_planner'))
```
You may also want to install [Gurobi](https://www.gurobi.com/) for Matlab to use it as MILP solver.

## Execution
The main script, which implements and solves the MILP formulation, is [optimalTaskAllocator.m](src/optimalTaskAllocator.m).
Inside this file there is a function defined as:
```
function [sol, fval, solving_time, dv_start_length] = optimalTaskAllocator(scenario_id, execution_id, scenario_size, formulation_variants_flags, config_flags)
```
First, the `scenario\_id` parameter should be numeric if we want to solve a manually predefined [scenario](scripts/scenario.m), `0` if we want to solve a randomly generated scenario, and should be not numeric if we want to solve a saved scenario. In this last case `scenario\_id` should be the ID name string of the saved scenario.

Second, the `execution_id` parameter is a string, used in the log file and as file sufix to save the scenario information.

Third, the `scenario size` parameter is a 1x3 vector with the number of robots (A), number of tasks (T) (without counting the recharge task) and number of different types of robots (types). This is used only when we want to solve a randomly generated scenario (`scenario_id == 0`), otherwise, this parameter can be set to an empty array.

Then, optionally, `formulation_variants_flags` parameter allows us to modify the problem to solve a scenario with a slightly different MILP formulation. It's a 1x4 logic vector corresponding to the following flags: `[recharges_allowed_flag, relays_allowed_flag, fragmentation_allowed_flag, variable_number_of_robots_allowed_flag]`. We can set this parameter to an empty array to use default config: the complete formulation. 

Last, `config_flags` parameter is a 1x7 logic vector corresponding to the following option flags: `[save_flag, test_flag, solve_flag, recovery_flag, display_flag, log_file_flag, print_solution_flag]`. The default configuration is used if this is set to an empty array.

In this way, we can run the planner to solve a ramdomly generated scenario with `3` robots, `2` tasks and `1` defferent
type of robot like:
```
[sol, fval, solving_time, dv_start_length] = optimalTaskAllocator(0, 'random', [3 2 1]);
```
By default, the print solution flag is set to false. To change that we need to specify it in the last parameter:
```
[sol, fval, solving_time, dv_start_length] = optimalTaskAllocator(0, 'random', [3 2 1], [], [1 1 0 0 0 0 0]);
```
To solve a predefined scenario (for example scenario `1`), and moreover to do so with a variant of the formulation, we can run it like:
```
[sol, fval, solving_time, dv_start_length] = optimalTaskAllocator(1, 'Test', [], [1 0 0 0], []);
```

### Analyzing the solution
After running the `optimalTaskAllocator()` function, we can check the values of some variables of the solution vector using the [getVarValue](scripts/getVarValue.m) script. To do so, we first need to load the information about the scenario that has just been solve.
**Note**: if it was a randomly generated scenario, remember to set the save flag on before when calling the main function.
```
[Agent, Task, A, N, T, S, R, Td_a_t_t, Te_t_nf] = scenario('scenario_id');
```
Where `scenario_id` is the id of the scenario just solved.

Now the `Agent` and `Task` structures with the scenario information are loaded. We can see also `A`, that is the number of
robots, `T`, the number of tasks including the Recharge task, `S`, the length of the robot's queue, `N`, maximum estimated number of fragments, `Td\_a\_t\_t` is a `A`x`T`x`T+1` array, `Td\_a\_t\_t(a, t2, t1)` would be the time it takes for robot `a` to move from task `t1`location to task `t2` location, and `Te\_t\_nf` is a `T`x`N` array, where `Te\_t\_nf(t,nf)` would be the time it takes for task `t` to execute if it where divided into `nf` fragments.

With this information and the information returned by the `optimalTaskAllocator()`, we can call `getVarValue()`. Lets
check for examples how mane fragments are task being divided into:
```
nf_t = getVarValue(display_flag, sol, A, N, T, S, dv_start_length, 'nf_t')
```
**Note**: on of the task will be the recharge task, ussually the first task.
The names of the variables can be consulted in both [optimalTaskAllocator.m](src/optimalTaskAllocator.m) and [getVarValue](scripts/getVarValue.m).

In case that we forgot to print the solution, we can also print it manually after loading the scenario information doing:
```
printSolution(sol, fval, Agent, Task, A, N, T, S, Td_a_t_t, Te_t_nf, dv_start_length, 'scenario_id', objective_function, predefined);
```

## References
<a id="1">[1]</a>
Gerkey, Brian P and Mataric, Maja J,
"A formal analysis and taxonomy of task allocation in multi-robot systems",
The International Journal of Robotics Research, vol. 23, no. 9, pp. 939–954, 2004.

<a id="2">[2]</a>
E. Nunes, M. Manner, H. Mitiche, and M. Gini,
“A taxonomy for task allocation problems with temporal and ordering constraints”,
Robotics and Autonomous Systems, vol. 90, pp. 55–70, 2017.

<a id="3">[3]</a>
G. A. Korsah, A. Stentz, and M. B. Dias,
“A comprehensive taxonomy for multi-robot task allocation”,
The International Journal of Robotics Research, vol. 32, no. 12, pp. 1495–1512, 2013.
