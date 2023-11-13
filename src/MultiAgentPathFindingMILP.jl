"""
    MultiAgentPathFindingMILP

A module for Multi-Agent Pathfinding with Mixed-Integer Linear Programming approach
"""
module MultiAgentPathFindingMILP

using Graphs: AbstractGraph, DiGraph, wheel_digraph
using Graphs: src, dst, edges, vertices, inneighbors, outneighbors, nv, ne, is_directed
using Graphs: add_vertex!, add_edge!, rem_edge!
using JuMP: Model, AffExpr
using JuMP: @variable, @constraint, @objective
using JuMP: set_silent, optimize!, termination_status, set_time_limit_sec
using JuMP: value, objective_value, fix, all_variables, set_start_value, add_to_expression!
using HiGHS: HiGHS
using MathOptInterface: OPTIMAL

export mapf_continuous_time, mapf_discrete_time
export mapf_continuous_time_dynamic_conflict

export MapfConfig, nv, ne, nagents
export parallel_lines,
    directional_star, grid_cross, line_overlap, wheel_pass, circular_ladder_pass

include("dynamic_continuous.jl")
include("conflict.jl")
include("continuous_time.jl")
include("discrete_time.jl")
include("config.jl")
include("case_generate.jl")

end
