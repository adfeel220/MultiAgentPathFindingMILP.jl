# Algorithms

```@docs
MultiAgentPathFindingMILP
```

## Table of Contents
```@contents
Pages=["algorithms.md"]
```

## Continuous-time

```@docs
mapf_continuous_time
MultiAgentPathFindingMILP.mapf_continuous_time!
MultiAgentPathFindingMILP.add_continuous_connectivity_constraints!
MultiAgentPathFindingMILP.add_continuous_timing_constraints!
MultiAgentPathFindingMILP.add_continuous_conflict_constraints!
```

## Dynamic Continuous-time

```@docs
mapf_continuous_time_dynamic_conflict
MultiAgentPathFindingMILP.parse_result
MultiAgentPathFindingMILP.parallel_shortest_path_result
MultiAgentPathFindingMILP.is_path_overlap
MultiAgentPathFindingMILP.add_edge_conflict!
MultiAgentPathFindingMILP.add_vertex_conflict!
MultiAgentPathFindingMILP.add_objective!
MultiAgentPathFindingMILP.right_align_get
```

Solve the continuous-time MAPF step-by-step. Starts from parallel shortest path. The conflict constraints are added dynamically whenever a conflict is detected.

## Discrete-time

```@docs
mapf_discrete_time
MultiAgentPathFindingMILP.mapf_discrete_time!
```

## Conflict Checking

```@docs
MultiAgentPathFindingMILP.check_overlap_on_vertex
MultiAgentPathFindingMILP.detect_vertex_conflict
MultiAgentPathFindingMILP.detect_edge_conflict
```