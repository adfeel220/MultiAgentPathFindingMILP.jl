
"""
    maph_continuous_time!(
        model,
        network, source_vertices, target_vertices, vertex_cost, edge_cost, departure_time;
        vertex_var_name, edge_var_name, integer,
    )

Modify a JuMP model by adding the variable, constraints and objective to compute continuous-time MAPH problem

# Arguments

- `model::Model`: `JuMP model` to be modified (adding variable, constraints, and objective)
- `network::AbstractGraph`: a directed graph representing the map
- `source_vertices::Vector{Int}`: an array of vertices indicating each agent's source vertex (the vertex an agent starts travel from)
- `target_vertices::Vector{Int}`: an array of vertices indicating each agent's target vertex (the vertex an agent end its travel at)
- `vertex_cost::Array{<:Real}`: costs for staying at each vertex. dimension = ([agent,] vertex)
- `edge_cost::Array{<:Real}`: costs for crossing each edge. dimension = ([agent,] vertex, vertex), we use (from_vertex, to_vertex) to indicate an edge

# Keyword arguments

- `vertex_var_name`: name of vertex selection variables, will be intepreted as a symbol
- `edge_var_name`: name of edge selection variables, will be intepreted as a symbol
- `integer::Bool`: whether to apply integer programming, apply linear relaxation otherwise

"""
function maph_continuous_time!(
    model::Model,
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    # dim: [agent,] vertex 
    vertex_cost::Array{T},
    # dim: [agent,] from_vertex, to_vertex
    edge_cost::Array{T},
    departure_time::Vector{Float64}=zeros(Float64, length(source_vertices));
    vertex_var_name=:vertex,
    edge_var_name=:edge,
    integer::Bool=true,
) where {T<:Real}
    @assert length(source_vertices) == length(target_vertices) "The number of source vertices does not match the number of target vertices"
    check_overlap_on_vertex(source_vertices, "Invalid source vertices for agents")
    check_overlap_on_vertex(target_vertices, "Invalid target vertices for agents")
    @assert 1 <= ndims(vertex_cost) <= 2 "Vertex cost can only be 2 dimensional (agent, vertex) or 1 (vertex), but get $(ndims(vertex_cost))-dimensions"
    @assert 2 <= ndims(edge_cost) <= 3 "Edge cost can only be 3 dimensional (agent, vertex, vertex) or 2 (vertex, vertex), but get $(ndims(edge_cost))-dimensions"
    @assert all(departure_time .>= zero(eltype(departure_time))) "Departure time must be non-negative"

    edge_tuples = [(src(ed), dst(ed)) for ed in edges(network)]
    n_agents = length(source_vertices)

    # Variables
    edge_select_vars = @variable(
        model, 0 <= x[a=1:n_agents, e=edge_tuples] <= 1; integer=integer
    )
    model[Symbol(edge_var_name)] = edge_select_vars

    vertex_select_vars = @variable(
        model, 0 <= y[a=1:n_agents, v=vertices(network)] <= 1; integer=integer
    )
    model[Symbol(vertex_var_name)] = vertex_select_vars

    ## Constraints
    for (agent_id, (agent_source, agent_target)) in
        enumerate(zip(source_vertices, target_vertices))

        # Agents start at their source vertex
        @constraint(model, vertex_select_vars[agent_id, agent_source] == 1)
        # Agents end at their target vertex
        @constraint(model, vertex_select_vars[agent_id, agent_target] == 1)

        # Agents leave from their sources
        @constraint(
            model,
            sum(
                edge_select_vars[agent_id, (agent_source, next_v)] for
                next_v in outneighbors(network, agent_source)
            ) - sum(
                edge_select_vars[agent_id, (prev_v, agent_source)] for
                prev_v in inneighbors(network, agent_source)
            ) == 1
        )
        # Agents arrive at their destination
        @constraint(
            model,
            sum(
                edge_select_vars[agent_id, (agent_target, next_v)] for
                next_v in outneighbors(network, agent_target)
            ) + sum(
                edge_select_vars[agent_id, (prev_v, agent_target)] for
                prev_v in inneighbors(network, agent_target)
            ) == 1
        )

        # Agents can only go out from a vertex if it goes in first
        for u in vertices(network)
            if u in (agent_source, agent_target)
                continue
            end
            @constraint(
                model,
                sum(
                    edge_select_vars[agent_id, (u, next_v)] for
                    next_v in outneighbors(network, u)
                ) - sum(
                    edge_select_vars[agent_id, (prev_v, u)] for
                    prev_v in inneighbors(network, u)
                ) == 0
            )
        end

        # If an agent travels via edge (u, v), it must stop at v
        for v in vertices(network)
            if v in agent_source
                continue
            end
            @constraint(
                model,
                vertex_select_vars[agent_id, v] ==
                    sum(edge_select_vars[agent_id, (u, v)] for u in inneighbors(network, v))
            )
        end
    end

    # Objective
    if ndims(edge_cost) == 3
        edge_objective = sum(
            edge_cost[agent_id, u, v] * edge_select_vars[agent_id, (u, v)] for
            (u, v) in edge_tuples, agent_id in 1:n_agents
        )
    else
        edge_objective = sum(
            edge_cost[u, v] * edge_select_vars[agent_id, (u, v)] for (u, v) in edge_tuples,
            agent_id in 1:n_agents
        )
    end

    if ndims(vertex_cost) == 2
        vertex_objective = sum(
            vertex_cost[agent_id, v] * vertex_select_vars[agent_id, v] for
            v in vertices(network), agent_id in 1:n_agents
        )
    else
        vertex_objective = sum(
            vertex_cost[v] * vertex_select_vars[agent_id, v] for v in vertices(network),
            agent_id in 1:n_agents
        )
    end

    @objective(model, Min, edge_objective + vertex_objective)

    return model
end

"""
    maph_continuous_time(
        network, source_vertices, target_vertices, vertex_cost, edge_cost, departure_time;
        integer, optimizer, silent
    )

Compute the MAPH problem in continuous time from a set of source vertices to target vertices.
Traversal of each vertex and edge comes with a cost.
Returns the selected vertices and edges for each agent

# Arguments

- `network::AbstractGraph`: a directed graph representing the map
- `source_vertices::Vector{Int}`: an array of vertices indicating each agent's source vertex (the vertex an agent starts travel from)
- `target_vertices::Vector{Int}`: an array of vertices indicating each agent's target vertex (the vertex an agent end its travel at)
- `vertex_cost::Array{<:Real}`: costs for staying at each vertex. dimension = ([agent,] vertex)
- `edge_cost::Array{<:Real}`: costs for crossing each edge. dimension = ([agent,] vertex, vertex), we use (from_vertex, to_vertex) to indicate an edge
- `departure_time::Vector{Float64}`: departure time of each agent (default is `zeros(Float64, length(source_vertices))`)

# Keyword arguments

- `integer::Bool`: whether the path should be integer-valued or real-valued (default is `true`)
- `optimizer`: JuMP-compatible solver (default is `HiGHS.Optimizer`)
- `silent::Bool`: turn of printing of model status printing

"""
function maph_continuous_time(
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    # dim: [agent,] vertex 
    vertex_cost::Array{T},
    # dim: [agent,] from_vertex, to_vertex
    edge_cost::Array{T},
    departure_time::Vector{Float64}=zeros(Float64, length(source_vertices));
    integer::Bool=true,
    optimizer=HiGHS.Optimizer,
    silent::Bool=true,
) where {T<:Real}
    model = Model(optimizer)
    if silent
        set_silent(model)
    end

    maph_continuous_time!(
        model,
        network,
        source_vertices,
        target_vertices,
        vertex_cost,
        edge_cost,
        departure_time;
        vertex_var_name=:vertex,
        edge_var_name=:edge,
        integer=integer,
    )
    optimize!(model)
    @assert termination_status(model) == OPTIMAL

    edge_selection_vars = value.(model[:edge])
    vertex_selection_vars = value.(model[:vertex])

    # parse vertices
    agents, selection = axes(vertex_selection_vars)
    valid_vertices = [
        [v for v in selection if vertex_selection_vars[agent_id, v] > 0.5] for
        agent_id in agents
    ]

    # parse edges
    agents, selection = axes(edge_selection_vars)
    valid_edges = [
        [ed for ed in selection if edge_selection_vars[agent_id, ed] > 0.5] for
        agent_id in agents
    ]

    return valid_vertices, valid_edges
end
