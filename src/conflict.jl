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
