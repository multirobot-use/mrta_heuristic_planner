## Multi-Robot Task Allocation (MRTA) for heterogeneous teams

This repository addresses a MRTA problem for heterogeneous teams. All code is written in Matlab and it includes: 1) a MILP (Mixed-Integer Linear Program) formulation of the problem, so it can be solved optimally using of-the-shelf solvers such as Gurobi; 2) a heuristic planer to compute efficiently approximate solutions; and 3) an algorithm to repair plans during mission execution in dynamic scenarios where robots may get delayed due to unexpected events. 

For a complete description of the problem and the algorithms, you can read our related publications: 

```bibtex
@inproceedings{calvo_icra24,
  address = {Yokohama, Japan},
  author = {Calvo, Alvaro and Capitan, Jesus},
  booktitle = {International Conference on Robotics and Automation (ICRA)},
  title = {{Optimal Task Allocation for Heterogeneous Multi-robot Teams with Battery Constraints}},
  pages = {7243-7249},
  year = {2024},
  doi={10.1109/ICRA57147.2024.10611147}
}
```

## Installation

There are no special requirements to be able to run the Matlab scripts.
Just make sure you have correctly added this project's path to the matlab path, including subfolders:

```matlab
addpath(genpath('<install_dir>/task_planner'))
savepath
```

You may also want to install [Gurobi](https://www.gurobi.com/) for Matlab to use it as MILP solver.

## Code description

The three main source codes are:

- [optimalTaskAllocator](src/optimalTaskAllocator.m): this script contains the complete MILP formulation. It receives a scenario coded into two data structures, i.e., robots and tasks, and returns the MILP solution array and its objective function value.
- [heuristicTaskAllocator](src/heuristicTaskAllocator.m): this script contains several heuristic algorithms that can solve the MRTA problem including the main heuristic version (number 9), a greedy version (number 12), a random version (number 6) and a pseudo-random version (number 4). It receives the scenario to solve coded into two data structures, and returns the solution coded inside the same data structures. It also returns information needed to repair the plan in case any delay occurs.
- [planRepair](src/planRepair.m): this code fix a delayed scenario if possible. It receives a plan solution contained in Robots and Tasks data structures, the synchronization and relays information, and the details about the amount and the location of the delay, and returns the plan, fixed or not, using the same data structures as in the input, and whether the plan was fixed or not.

Other useful scripts are:

- [printSolution](scripts/printSolution.m): this script can be used to print the solution to a scenario either solved with *optimalTaskAllocator*, *heuristicTaskAllocator* or *planRepair*. It uses a color code to represent displacement, waiting and execution times (blue, yellow and green respectively). Consecutive fragments of the same task in the same robot can be joined into a single bar or not using an input flag.
- [generateScenarios](scripts/generateScenarios.m): this script can be used to create a dataset of scenarios. Through the input variables you can specify the amount of scenarios to generate, the number of agents they must contain, the number of tasks per scenario, the amount of different types of robots and if the scenarios should be discretized or not.
- [previewScenario](scripts/previewScenario.m): this scrip displays the inputted scenario using bars so you can visualize all robots' and tasks' relevant information.
- [scenario](scripts/scenario.m): this script can be used to load a handmade scenario, to load a randomly generated scenario using its ID, or to randomly generate a single scenario with some specified constraints.

## Execution examples

### Compute exact solutions using the MILP formulation

*optimalTaskAllocator* contains a function defined as:

```matlab
function [sol, fval] = optimalTaskAllocator(scenario_id, execution_id, scenario_size, objective_function, formulation_variants_flags, config_flags, solver_config)
```

First, the `scenario_id` parameter should be numeric if we want to solve a manually predefined [scenario](scripts/scenario.m), `0` if we want to solve a randomly generated scenario, and should be a string if we want to solve a saved scenario. In this last case `scenario_id` should be the ID name string of the saved scenario.

Second, the `execution_id` parameter is a string, used in the log file and as file suffix to save the scenario information.

Third, the `scenario size` parameter is a 1x3 vector with the number of robots (R), number of tasks (T) (without counting the recharge task) and amount of different types of robots (types). This is used only when we want to solve a randomly generated scenario (`scenario_id == 0`), otherwise, this parameter can be set to an empty array.

`objective_function` is an integer parameter that allows us to choose between different predefined objective functions to minimize. The details about the different options can be found commented in the source file.

Then, optionally, `formulation_variants_flags` parameter allows us to modify the problem to solve a scenario with a slightly different MILP formulation. It's a 1x4 logic vector corresponding to the following flags: `[recharges_allowed_flag, relays_allowed_flag, fragmentation_allowed_flag, variable_number_of_robots_allowed_flag]`. We can set this parameter to an empty array to use default configuration: the complete formulation.

Last, `config_flags` parameter is a 1x9 logic vector corresponding to the following option flags: `[solve_flag, initialize_flag, genetic_algorithm_solver, print_solution_flag, display_flag, log_file_flag, save_flag, test_flag, recovery_flag]`. The default configuration is used if this is set to an empty array.

In this way, we can run the planner to solve a randomly generated scenario with `3` robots, `2` tasks and `1` different type of robot like:

```matlab
[sol, fval] = optimalTaskAllocator(0, 'random', [3 2 1]);
```

By default, the print solution flag is set to false. To change that we need to specify it in the last parameter:

```matlab
[sol, fval] = optimalTaskAllocator(0, 'random', [3 2 1], [], [], [1 1 0 0 0 0 0]);
```

If the scenario is feasible, we would see the optimal solution displayed in a bar plot like this:

![random3x2](https://github.com/user-attachments/assets/d3c812c1-3c76-4f04-8fd4-a0d46746e3e6)

Note that the bar for each task is divided into two parts. The upper part represents the total duration of the task. We can see that the task name/id appears above it. The lower part represents which part of task duration corresponds to the traveling time, waiting time and execution time, represented in blue, yellow and green respectively.

To solve a pre-generated scenario (for example scenario `3r2t1`), and moreover to do so with a different objective function and with a variant of the formulation, we can run it like:

```matlab
[sol, fval] = optimalTaskAllocator('3r2t1', 'Test', [], 4, [1 0 0 0], []);
```

Here we are running the mission planner with the fourth predefined objective function and a variant where only recharge are allowed. Task are not fragmentable nor decomposable, and the coalition size flexibility is always hard.

Last, we can use the `genetic_algorithm_solver` configuration parameter to solve the MILP problem using [Matlab's GA solver](https://es.mathworks.com/help/gads/ga.html) from the Optimization Toolbox.

```matlab
config_flags = [1, 0, 1, 1, 1, 0, 0, 0, 0];
[sol, fval] = optimalTaskAllocator(scenario_id, execution_id, [], objective_function, [], config_flags, 0);
```

While running, you should be able to see a plot like this:

<img src="https://github.com/alvcalmat/XATS/assets/74324102/4c577bf4-cb13-461c-839b-733d2a0f05d7" width="600"/>

To know more about GA solver options go to [Genetic Algorithm Options](https://es.mathworks.com/help/gads/genetic-algorithm-options.html).

### Analyzing exact solutions

After running the `optimalTaskAllocator()` function, we can check the values of some variables of the solution vector using the [getVarValue](scripts/getVarValue.m) script. To do so, we first need to load the information about the scenario that has just been solved:

**Note**: if it was a randomly generated scenario, remember to set the save flag on before when calling the main function.

```matlab
[Agent, Task] = scenario(scenario_id);
```

Where `scenario_id` is the id of the scenario just solved.

Now the `Agent` and `Task` structures with the scenario information are loaded.

With this information and the solution vector returned by the `optimalTaskAllocator()`, we can call `getVarValue()` to check the value of any decision variable that we want. To check for example how many fragments are tasks being divided into:

```matlab
nf_t = getVarValue(sol, Agent, Task, 'nf_t')
```

**Note**: one of the tasks will be the recharge task, usually the first task.

The names of the variables can be consulted in the [getVarValue](scripts/getVarValue.m) source code.

In case that we forgot to print the solution, we can also print it manually after loading the planner workspace information by doing:

```matlab
printSolution(sol, Agent, Task, 0, 'scenario_id', 'execution_id', fval);
```

### Compute estimated solutions using the heuristic solver

The heuristic planner creates, if feasible, a solution to the specified scenario. This algorithm is contained in [heuristicTaskAllocator](src/heuristicTaskAllocator.m) and shall be defined in a different way depending on the use we are giving it.

To use the heuristic solution as initial solution for the MILP formulation this function shall be defined as:

```matlab
function [Agent, Task, allocation_order, S_R] = heuristicTaskAllocator(arg_1, arg_2, reward_coefficients, version, seed)
```

**Note**: This heuristic method can be used to initialize the MILP solver by setting the corresponding config flag at MILP solver's launching time.

If you think you may need to call the plan repair function (e.g. in a real live execution), we need extra information from this function, so it shall be defined as:

```matlab
function [Agent, Task, Synchs, Relays] = heuristicTaskAllocator(arg_1, arg_2, reward_coefficients, version, seed)
```

Last, the simplest and fastest heuristic planner execution is achieved by defining it as:

```matlab
function [Agent, Task, allocation_order] = heuristicTaskAllocator(arg_1, arg_2, reward_coefficients, version, seed)
```

To execute `heuristicTaskAllocator()`, `arg_1` and `arg_2` should either be Agent and  structures or a valid scenario id and optionally execution id. The second parameter, `reward_coefficients`, controls the task allocation order depending on the solver version. `version` is used to select the method to get the task allocation order. The 9th version is the main heuristic algorithm, 12th is a greedy version, 6th a random solver version, and 4th a pseudo-random version. This four main heuristic versions are explain deeper in [[2]](#2). Last, `seed` is used as task allocation order in the "seed" version (input order).
The solution to the specified scenario is added to `Agent` and `Tasks` structures.

The next gif shows an example of the heuristic process of building a solution for a scenario with 10 heterogeneous robots and 50 tasks.

![10a50t1](https://github.com/alvcalmat/XATS/assets/74324102/79c46fe3-3675-4173-a47e-d84a3c705b80)

### Plan repair algorithm

When a plan is no longer valid because of a delay, we can use the plan repair function to try to fix the plan in order to avoid repeating the whole planning process. This function is defined as:

```matlab
function [Agent, Task, result] = planRepair(Agent, Task, Synchs, Relays, delay)
```

As input we have: the robot structure array `Agent`, the task structure array `Task`, a list of the slots to be synchronized `synch`, a list of relays to be coordinated `relays`, and an array containing the amount of delay in seconds and the slot delayed `[d r s]`. As output we have the fixed plan inside of the same `Agent` and `Task` structures, and a flag indicating wether the plan was fixable or not.

## References

<a id="3">[3]</a>
Gerkey, Brian P and Mataric, Maja J,
"A formal analysis and taxonomy of task allocation in multi-robot systems",
The International Journal of Robotics Research, vol. 23, no. 9, pp. 939–954, 2004.

<a id="4">[4]</a>
E. Nunes, M. Manner, H. Mitiche, and M. Gini,
“A taxonomy for task allocation problems with temporal and ordering constraints”,
Robotics and Autonomous Systems, vol. 90, pp. 55–70, 2017.

<a id="5">[5]</a>
G. A. Korsah, A. Stentz, and M. B. Dias,
“A comprehensive taxonomy for multi-robot task allocation”,
The International Journal of Robotics Research, vol. 32, no. 12, pp. 1495–1512, 2013.
