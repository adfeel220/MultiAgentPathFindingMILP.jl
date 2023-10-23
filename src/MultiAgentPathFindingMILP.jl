"""
    MultiAgentPathFindingMILP

A module for Multi-Agent Pathfinding with Mixed-Integer Linear Programming approach
"""
module MultiAgentPathFindingMILP

using Graphs: AbstractGraph
using Graphs: weights, nv, src, dst, edges, vertices, inneighbors, outneighbors
using JuMP: Model
using JuMP: @variable, @constraint, @objective


include("conflict.jl")
include("continuous_time.jl")

end
