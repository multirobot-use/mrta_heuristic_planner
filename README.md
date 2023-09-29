# Optimal Task Allocation for Heterogeneous Multi-robot Teams with Battery Constraints
## Overview
This repository deals with optimal task planning missions with heterogeneous teams of robots that will have to execute a
number of tasks that may have different specifications and types, while dealing with limited resources that can be
recovered by executing certain actions, i.e. the robots have limited battery power, but have the possibility to plan
recharges.

The objective is to find a mission plan, i.e. to assign all tasks to the robots (also called agents) needed to complete them successfully respecting the constraints imposed by each task, while minimizing the total execution time, i.e. the makespan.

The problem is formulated using Mixed Integer Linear Programming (MILP) in Matlab. To solve it, the `intlinprog()`
Matlab function is called. If you have Gurobi installed in Matlab, this function should be overwritten by the
`intlinprog()` Gurobi function, so the same code works using Matlab or Gurobi solvers.

## Problem description
The proposed problem constitutes a first step towards the formulation and analysis of a problem of complexity never before addressed in the state of the art.

The problem can be classified as ST-MR-TA according to the taxonomy described in [[1]](#1), i.e., robots can execute only one task at a time (ST), tasks may need more than one robot to execute (MR), and robots are assigned tasks to be executed according to a schedule. Can also be classified as Task Windows (TW) according to [[2]](#2). There's also another category in that taxonomy, Synchronization and Precedence (SP). We do not have precedence constraints, but we do have synchronization constraints when multiple robots has to execute a task together or do a relay due to battery constraints. Last, this problem can be classified also as CD according to [[3]](#3), i.e. the tasks has complex inter-task dependencies.

Robots are mainly characterized by their type. Tasks have a list of robot types that have hardware compatible with it. In this way, the formulation can restrict the assignment of tasks to task-compatible robots only, being totally agnostic to the actual hardware that constitutes each type of robot.

To understand the wide variety of tasks that are contemplated within the problem, we talk about two characteristics of the tasks: composability and number of robots. Depending on the composability, tasks can be non-decomposable, relayable or fragmentable. Non-composable tasks must be executed continuously by the same robots, relayable tasks must be executed continuously, but allow relays between several robots, and finally, fragmentable tasks can be executed in fragments that do not have to be coordinated between them and can be spread over the schedule of one or several robots. On the other hand, tasks can have a fixed number of robots, i.e. they need to be executed by exactly the specified number of robots, they can have a variable number of robots, meaning that they specify an ideal number of robots with which to be executed, but allow a different number of robots to be used, or they can specify no number of robots at all, and their duration will depend on the number of robots finally employed to execute them. Tasks can belong to any type that is a combination of these two characteristics.

In the [scenario](scripts/scenario.m) script you can see in more detail how agents (robots) and tasks are implemented within the formulation.

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
function [sol, fval] = optimalTaskAllocator(scenario_id, execution_id, scenario_size, formulation_variants_flags, config_flags)
```
First, the `scenario_id` parameter should be numeric if we want to solve a manually predefined [scenario](scripts/scenario.m), `0` if we want to solve a randomly generated scenario, and should be a string if we want to solve a saved scenario. In this last case `scenario_id` should be the ID name string of the saved scenario.

Second, the `execution_id` parameter is a string, used in the log file and as file suffix to save the scenario information.

Third, the `scenario size` parameter is a 1x3 vector with the number of robots (A), number of tasks (T) (without counting the recharge task) and number of different types of robots (types). This is used only when we want to solve a randomly generated scenario (`scenario_id == 0`), otherwise, this parameter can be set to an empty array.

Then, optionally, `formulation_variants_flags` parameter allows us to modify the problem to solve a scenario with a slightly different MILP formulation. It's a 1x4 logic vector corresponding to the following flags: `[recharges_allowed_flag, relays_allowed_flag, fragmentation_allowed_flag, variable_number_of_robots_allowed_flag]`. We can set this parameter to an empty array to use default config: the complete formulation. 

Last, `config_flags` parameter is a 1x7 logic vector corresponding to the following option flags: `[save_flag, test_flag, solve_flag, recovery_flag, display_flag, log_file_flag, print_solution_flag]`. The default configuration is used if this is set to an empty array.

In this way, we can run the planner to solve a ramdomly generated scenario with `3` robots, `2` tasks and `1` defferent
type of robot like:
```
[sol, fval] = optimalTaskAllocator(0, 'random', [3 2 1]);
```
By default, the print solution flag is set to false. To change that we need to specify it in the last parameter:
```
[sol, fval] = optimalTaskAllocator(0, 'random', [3 2 1], [], [1 1 0 0 0 0 0]);
```
If the scenario is feasible, we would see the optimal solution displayed in a bar plot like this:

![random3x2](https://github.com/multirobot-use/task_planner/assets/74324102/cc9a8771-5bb3-42b3-a7db-ae48c7c4df03)

Note that the bar for each task is divided into two parts. The upper part represents the total duration of the task. We can see that the task name/id appears above it. The length of the lower part represents what part of task time corresponds to the travel time, waiting time and execution time.

To solve a predefined scenario (for example scenario `1`), and moreover to do so with a variant of the formulation, we can run it like:
```
[sol, fval] = optimalTaskAllocator(1, 'Test', [], [1 0 0 0], []);
```

### Analyzing the solution
After running the `optimalTaskAllocator()` function, we can check the values of some variables of the solution vector using the [getVarValue](scripts/getVarValue.m) script. To do so, we first need to load the information about the scenario that has just been solved.
If we had the save flag on when we solved it, there should be a `planner_workspace.mat` file in the `mat` folder corresponding to the variables from the `optimalTaskAllocator()` function that we need to properly analyze the solution. If we forgot to turned it on, we can just call the planner again with the test flag on specifying to solve the last scenario. This will just save the workspace information and return. Then to load the information we just run the following:

**Note**: if it was a randomly generated scenario, remember to set the save flag on before when calling the main function.
```
load('../mat/planner_workspace.mat');
```
Where `scenario_id` is the id of the scenario just solved.

Now the `Agent` and `Task` structures with the scenario information are loaded. We can see also `A`, that is the number of
robots, `T`, the number of tasks including the Recharge task, `S`, the length of the robot's queue, `N`, maximum estimated number of fragments, `Td_a_t_t` is a `A`x`T`x`T+1` array, `Td_a_t_t(a, t2, t1)` would be the time it takes for robot `a` to move from task `t1`location to task `t2` location, and `Te_t_nf` is a `T`x`N` array, where `Te_t_nf(t,nf)` would be the time it takes for task `t` to execute if it where divided into `nf` fragments. `dv_start_length` is a structure containing the starting position and length of each decision variable in the solution vector (`sol`).

With this information and the information returned by the `optimalTaskAllocator()`, we can call `getVarValue()` to check the value of any decision variable that we want. To check for example how mane fragments are tasks being divided into:
```
nf_t = getVarValue(display_flag, sol, A, N, T, S, dv_start_length, 'nf_t')
```
**Note**: on of the tasks will be the recharge task, usually the first task.

The names of the variables can be consulted in both [optimalTaskAllocator.m](src/optimalTaskAllocator.m) and [getVarValue](scripts/getVarValue.m) source codes.

In case that we forgot to print the solution, we can also print it manually after loading the planner workspace information doing:
```
printSolution(sol, fval, Agent, Task, dv_start_length, 'scenario_id', objective_function, predefined);
```

## Check solution
There also available a script, [checkSolution.m](scripts/checkSolution.m), that can be used either to check if the result if correct, if any of the soft constraints are being broken, or to manually propose a solution to the task allocation problem and see if it is a valid solution or not. This will be used in the future also to check if the heuristic proposed solutions are valid or not.

To see an example, lets say we want to check if a handmade solution for the predefined scenario number `5` is valid or not. The first step would be to load into the workspace the scenario information:
```
[sol, fval] = optimalTaskAllocator(5, 'checkSolutionTest', [], [], [0, 0, 0, 0, 0, 1, 0]);
load('../mat/planner_workspace.mat');
```

Then we need to build the solution itself and put the main decision variables into a vector:
```
% Build x(a,t,s) by hand
xats = zeros(A,(T+1),(S+1));

% Initial tasks
t = 0;
s = 0;
xats(:, t + 1, s + 1) = 1;

% Agent 1
a = 1;
tasks = [2, 0, 2];
s = 0;
for task = 1:length(tasks)
    t = tasks(task);
    s = s + 1;
    xats(a, t + 2, s + 1) = 1;
end

% Agent 2
a = 2;
tasks = [1, 2];
s = 0;
for task = 1:length(tasks)
    t = tasks(task);
    s = s + 1;
    xats(a, t + 2, s + 1) = 1;
end

xats = reshape(xats,1,[]);

% nf(t) by hand
nf_t = 1 * ones(1,T);
nf_t(2 + 1) = 3;

% S by hand: for each slot, the number of synchronizations should be ns(t,a1,s1) = (na(t) - 1) * x(a1,t,s1)
Sta1s1a2s2 = zeros(T-1,A,S,A,S);
Sta1s1a2s2 = reshape(Sta1s1a2s2,1,[]);

% R by hand: the total number of relays per task should be nr(t) = n(t) - na(t)
Rta1s1a2s2 = zeros(T-1,A,S,A,S);
Rta1s1a2s2(2, 1,  1, 2,  2) = 1;
Rta1s1a2s2(2, 2,  2, 1,  3) = 1;
Rta1s1a2s2 = reshape(Rta1s1a2s2,1,[]);

% xats_nf_S_R
xats_nf_S_R = [xats nf_t Sta1s1a2s2 Rta1s1a2s2];
```

Note that `xats` is the task allocation itself. `x(a,t,s) = 1` if task `t` is assigned to slot `s` of robot `a`, `0` otherwise. `nf(t)` is the numbers of fragments that task `t` is divided into. Last `S(t, a1, s1, a2, s2)` and `R(t, a1, s1, a2, s2)` are the synchronization and relays variables, `1` if there is a coordination between a fragment of task `t` allocated in slot `s1` of agent `a1` and a fragment of task `t` assigned to slot `s2` of agent `a2`, `0` otherwise.

Last, we need to call the `checkSolution()` function:
```
[dv_chk, fval_chk, result_chk] = checkSolution(xats_nf_S_R, A, N, T, S, dv_start_length, Task, Agent, Td_a_t_t, Te_t_nf, Ft_saf, H_a_t, 'tmp', 1, z_max, tfin_max, Tw_max, U_max, d_tmax_max, s_used_max);
```

We should see the solution in a bar plot like the following one:

![scenario5](https://github.com/multirobot-use/task_planner/assets/74324102/8c906833-db7f-49fa-91f6-fcf2c05176f7)

To check a solution obtained with the solver we use the same command but changing `xats_nf_S_R` by `sol`:
Last, we need to call the `checkSolution()` function:
```
[dv_chk, fval_chk, result_chk] = checkSolution(sol, A, N, T, S, dv_start_length, Task, Agent, Td_a_t_t, Te_t_nf, Ft_saf, H_a_t, 'tmp', 1, z_max, tfin_max, Tw_max, U_max, d_tmax_max, s_used_max);
```

**Note** that there may be more that one optimal solution, and as a consequence of that, the showed solution may be an equivalent solution to the one founded by the solver. Mainly, the tasks where the coordination waiting times are applied may change.

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
