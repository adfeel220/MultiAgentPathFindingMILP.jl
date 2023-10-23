"""
    MultiAgentPathFindingMILP

A module for Multi-Agent Pathfinding with Mixed-Integer Linear Programming approach
"""
module MultiAgentPathFindingMILP

using Graphs: AbstractGraph
using Graphs: src, dst, edges, vertices, inneighbors, outneighbors
using JuMP: Model
using JuMP: @variable, @constraint, @objective
using JuMP: set_silent, optimize!, termination_status, value
using HiGHS: HiGHS
using MathOptInterface: OPTIMAL

export maph_continuous_time

include("conflict.jl")
include("continuous_time.jl")

end
