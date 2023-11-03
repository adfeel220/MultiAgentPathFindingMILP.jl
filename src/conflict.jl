DEFAULT_VERTEX_CONFLICT_SIGNAL = (v=-1, a1=-1, a2=-1)
DEFAULT_EDGE_CONFLICT_SIGNAL = (e=(-1, -1), a1=-1, a2=-1, swap=false)

"""
    check_overlap_on_vertex(
        agent_location, msg="";
        raise_assertion=false
    )

# Arguments

- `agent_location`: the location of agents in terms of vertices as an iterable,
where the indices are the agent IDs and the values are the IDs of the vertices they occupy
- `msg::String`: A message as prefix before the default error message, by defualt ""

# Keyword arguments

- `raise_assertion::Bool`: whether to raise an assertion error when overlapping agents are detected

"""
function check_overlap_on_vertex(
    agent_location, msg::String=""; raise_assertion::Bool=false
)::Bool
    is_overlap = length(Set(agent_location)) < length(agent_location)

    # If no error is required to be generated
    if !raise_assertion
        return is_overlap
    end

    # No overlap, no error generated
    if !is_overlap
        return false
    end

    # Has overlap and needs to generate error message
    vertex_occupancy = Dict(vertex => Vector{Int}(undef, 0) for vertex in agent_location)

    for (agent_id, vertex) in enumerate(agent_location)
        push!(vertex_occupancy[vertex], agent_id)
    end

    # Select the entries with overlapping agents
    overlap_agents = Dict(
        vertex => agents for (vertex, agents) in vertex_occupancy if length(agents) > 1
    )

    # Generate error message
    msg *= length(msg) > 0 ? ". " : ""
    message = join(
        ["Agent $agents occupy vertex $vertex" for (vertex, agents) in overlap_agents], ", "
    )

    throw(AssertionError("$(msg)Multiple agents cannot occupy the same vertex. $message."))
    return true
end

"""
    detect_vertex_conflict(
        vertices_timing, edges_timing; epsilon
    )
Return a NamedTuple of value `(v=vertex, a1=agent1, a2=agent2)` indicating the first detected
vertex conflict between `agent1` and `agent2` on `vertex`. Return default value of all `-1` if
no conflict is detected.

# Arguments

- `vertices_timing::Vector{Vector{Tuple{T,Int}}}`: The timing on vertex traversal of each agent,
`vertices_timing[agent][nth_hop] = (time_of_arrival, vertex)`
- `edges_timing::Vector{Vector{Tuple{T,Tuple{Int,Int}}}}`: The timing on edge traversal of each agent,
`edges_timing[agent][nth_hop] = (time_of_arrival, (from_vertex, to_vertex))`

# Keyword arguments

- `epsilon::{T<:Real}`: a small value to distinguish floating point equality, by default `1e-9`
"""
function detect_vertex_conflict(
    vertices_timing::Vector{Vector{Tuple{T,Int}}},
    edges_timing::Vector{Vector{Tuple{T,Tuple{Int,Int}}}};
    epsilon::T=1e-9,
)::NamedTuple where {T<:Real}

    # At each vertex, the occupancy of agent as (is_start, agent_id, timestamp)
    vertex_occupancy = Dict{Int,Vector{Tuple{Bool,Int,Float64}}}()

    for (agent_id, itinerary) in enumerate(vertices_timing)
        for (step_id, (timestamp, vertex)) in enumerate(itinerary)
            if step_id == length(itinerary)
                continue
            end

            finish_time, (from_v, to_v) = edges_timing[agent_id][step_id]
            if timestamp ≈ finish_time
                continue
            end

            if haskey(vertex_occupancy, vertex)
                push!(vertex_occupancy[vertex], (true, agent_id, timestamp))
            else
                vertex_occupancy[vertex] = [(true, agent_id, timestamp)]
            end

            @assert from_v == vertex "path not connected for agent $agent_id on vertex $vertex"

            push!(vertex_occupancy[from_v], (false, agent_id, finish_time - epsilon))
        end
    end

    # Check if any violation, i.e. for any vertex, later agent must enter after previous agent leaves
    for (vertex, occupancy) in vertex_occupancy
        sort!(occupancy; by=(x -> x[3]))

        for event_id in 1:2:(length(occupancy) - 1)
            is_start1, agent1, timestamp1 = occupancy[event_id]
            is_start2, agent2, timestamp2 = occupancy[event_id + 1]

            # agent2 enters vertex while agent1 has not left yet
            if is_start1 == is_start2
                return (v=vertex, a1=agent1, a2=agent2)
            end
        end
    end

    return DEFAULT_VERTEX_CONFLICT_SIGNAL
end

"""
    detect_edge_conflict(
        vertices_timing, edges_timing; detect_swap, epsilon
    )
Return a NamedTuple of value `(e=edge_tuple, a1=agent1, a2=agent2, swap=is_swap)` indicating the first detected
edge conflict between `agent1` and `agent2` on `edge_tuple`. The violation is a swap conflict if `swap` is `true`.
Return default value of all `-1` if no conflict is detected.

# Arguments

- `vertices_timing::Vector{Vector{Tuple{T,Int}}}`: The timing on vertex traversal of each agent,
`vertices_timing[agent][nth_hop] = (time_of_arrival, vertex)`
- `edges_timing::Vector{Vector{Tuple{T,Tuple{Int,Int}}}}`: The timing on edge traversal of each agent,
`edges_timing[agent][nth_hop] = (time_of_arrival, (from_vertex, to_vertex))`

# Keyword arguments

- `detect_swap::Bool`: whether to detect edge swapping conflict, by default true
- `epsilon::{T<:Real}`: a small value to distinguish floating point equality, by default `1e-9`
"""
function detect_edge_conflict(
    vertices_timing::Vector{Vector{Tuple{T,Int}}},
    edges_timing::Vector{Vector{Tuple{T,Tuple{Int,Int}}}};
    detect_swap::Bool=true,
    epsilon::T=1e-9,
)::NamedTuple where {T<:Real}

    # At each edge, the occupancy of agent as (is_start, agent_id, timestamp, is_inverted)
    edge_occupancy = Dict{Tuple{Int,Int},Vector{Tuple{Bool,Int,Float64,Bool}}}()

    for (agent_id, itinerary) in enumerate(edges_timing)
        for (step_id, (timestamp, edge)) in enumerate(itinerary)
            inverted = false
            if detect_swap
                # keep the ascending order of edge representation to detect swapping effect
                if edge[1] > edge[2]
                    edge = (edge[2], edge[1])
                    inverted = true
                end
            end
            finish_time, vertex = vertices_timing[agent_id][step_id + 1]

            if haskey(edge_occupancy, edge)
                push!(edge_occupancy[edge], (true, agent_id, timestamp, inverted))
            else
                edge_occupancy[edge] = [(true, agent_id, timestamp, inverted)]
            end

            push!(edge_occupancy[edge], (false, agent_id, finish_time - epsilon, inverted))
        end
    end

    # Check if any violation, i.e. for any edge, later agent must enter after previous agent leaves
    for (edge, occupancy) in edge_occupancy
        sort!(occupancy; by=(x -> x[3]))

        for event_id in 1:2:(length(occupancy) - 1)
            is_start1, agent1, timestamp1, is_inverted1 = occupancy[event_id]
            is_start2, agent2, timestamp2, is_inverted2 = occupancy[event_id + 1]

            # agent2 enters edge while agent1 has not left yet
            if is_start1 == is_start2
                return (e=edge, a1=agent1, a2=agent2, swap=is_inverted1 != is_inverted2)
            end
        end
    end

    return DEFAULT_EDGE_CONFLICT_SIGNAL
end
