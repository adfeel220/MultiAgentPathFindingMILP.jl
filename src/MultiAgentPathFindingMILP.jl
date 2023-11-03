"""
    MultiAgentPathFindingMILP

A module for Multi-Agent Pathfinding with Mixed-Integer Linear Programming approach
"""
module MultiAgentPathFindingMILP

using Graphs: AbstractGraph
using Graphs: src, dst, edges, vertices, inneighbors, outneighbors, nv, ne, is_directed
using JuMP: Model
using JuMP: @variable, @constraint, @objective
using JuMP: set_silent, optimize!, termination_status, set_time_limit_sec
using JuMP: value, objective_value
using HiGHS: HiGHS
using MathOptInterface: OPTIMAL

export mapf_continuous_time, mapf_discrete_time

include("conflict.jl")
include("continuous_time.jl")
include("discrete_time.jl")

end
