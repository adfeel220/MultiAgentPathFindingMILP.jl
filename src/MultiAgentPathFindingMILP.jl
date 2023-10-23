"""
    MultiAgentPathFindingMILP

A module for Multi-Agent Pathfinding with Mixed-Integer Linear Programming approach
"""
module MultiAgentPathFindingMILP

using Graphs: AbstractGraph
using Graphs: weights, nv
using JuMP: Model


include("conflict.jl")
include("continuous_time.jl")

end
