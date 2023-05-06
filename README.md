# Traveling Salesman Problem with Drone
This repository contains algorithms for solving Traveling Salesman Problem with Drone (TSP-D). TSP-D is a new planning problem that combine a truck and a drone to make effective deliveries in the near future. There are several types of heuristics and exact algorithms. Algorithms are coded with Python3. You can find exact or heuristic solutions of TSP-D and also can view the delivery routes of the truck and the drone. These were used in my research for bachelor's degree at Tokyo Institute of Technology in 2020.

## Finding exact solutions
DP for TSP-D was proposed by Bouman, Agatz and Schmidt.
```
└─exact_algorithms
    │  DP.py
```

## Finding heuristic solutions
Two kinds of heuristic algorithms for TSP-D was proposed by Agatz, Bouman, and Schmidt. Also there are some procedures to improve the outputs of the heuristics.

```
└─heuristics
    │  exact_partitioning.py
    |  greedy_partitioning.py
    |  iterative_improvement_procedure.py
```

## Other functions

There are some python files that define other functions that is needed to run the main files.

```
└─other_functions
    |  basic_functions.py
    |  generating_functions.py
    |  route_drawing.py
    |  solving_TSP.py
```



## References
P. Bouman, N. Agatz, and M. Schmidt. “Dynamic programming approaches for the traveling
salesman problem with drone.” Networks 72(2018), 528-542.

N. Agatz, P. Bouman, and M. Schmidt. “Optimization approaches for the traveling salesman
problem with drone.” Transportation Science 52(2018), 965-981
