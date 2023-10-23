
"""
    maph_continuous_time!(
        model,
        network, source_vertices, target_vertices, edge_cost;
        var_name, integer,
    )

Modify a JuMP model by adding the variable, constraints and objective to compute continuous-time MAPH problem
"""
function maph_continuous_time!(
    model::Model,
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    # dim: [agent,] vertex 
    vertex_cost::Array{T}=zeros(Float64, (length(source_vertices), nv(network))),
    # dim: [agent,] from_vertex, to_vertex
    edge_cost::Array{T}=weights(network);
    edge_var_name,
    vertex_var_name,
    integer::Bool=true,
) where {T<:Real}
    @assert length(source_vertices) == length(target_vertices) "The number of source vertices does not match the number of target vertices"
    check_overlap_on_vertex(source_vertices, "Invalid source vertices for agents")
    check_overlap_on_vertex(target_vertices, "Invalid target vertices for agents")
    @assert 1 <= ndims(vertex_cost) <= 2 "Vertex cost can only be 2 dimensional (agent, vertex) or 1 (vertex), but get $(ndims(vertex_cost))-dimensions"
    @assert 2 <= ndims(edge_cost) <= 3 "Edge cost can only be 3 dimensional (agent, vertex, vertex) or 2 (vertex, vertex), but get $(ndims(edge_cost))-dimensions"

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
            @constraint(
                model,
                vertex_select_vars[agent_id, v] >=
                    sum(edge_select_vars[agent_id, (u, v)] for u in inneighbors(network, v))
            )
        end
    end

    # Objective
    if ndims(edge_cost) == 3
        edge_objective = sum(
            sum(
                edge_cost[agent_id, u, v] * edge_select_vars[agent_id, (u, v)] for
                (u, v) in edge_tuples
            ) for agent_id in 1:n_agents
        )
    else
        edge_objective = sum(
            sum(
                edge_cost[u, v] * edge_select_vars[agent_id, (u, v)] for
                (u, v) in edge_tuples
            ) for agent_id in 1:n_agents
        )
    end

    if ndims(vertex_cost) == 2
        vertex_objective = sum(
            sum(
                vertex_cost[agent_id, v] * vertex_select_vars[agent_id, v] for
                v in vertices(network)
            ) for agent_id in 1:n_agents
        )
    else
        vertex_objective = sum(
            sum(
                vertex_cost[v] * vertex_select_vars[agent_id, v] for v in vertices(network)
            ) for agent_id in 1:n_agents
        )
    end

    @objective(model, Min, edge_objective + vertex_objective)

    return model
end
