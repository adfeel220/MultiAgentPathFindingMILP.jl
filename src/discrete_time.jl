
"""
    mapf_discrete_time!(
        model,
        network, source_vertices, target_vertices, vertex_cost, edge_cost, departure_time;
        time_duration, vertex_var_name, edge_var_name, integer,vertex_binding
    )

Modify a JuMP model by adding the variable, constraints and objective to compute discrete-time MAPH problem
"""
function mapf_discrete_time!(
    model::Model,
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    # dim: [agent,] vertex
    vertex_cost::AbstractArray,
    # dim: [agent,] from_vertex, to_vertex
    edge_cost::AbstractArray,
    departure_time::Vector{Int}=zeros(Int, nv(network));
    time_duration::Int=ne(network),
    vertex_var_name=:vertex,
    edge_var_name=:edge,
    integer::Bool=true,
    vertex_binding::Bool=false,
    vertex_visit::Symbol=:auto, # `:auto` needs to wait if vertex cost > 0, `:yes` means always needs to wait, `:no` means not enforced
)
    @assert length(source_vertices) == length(target_vertices) == length(departure_time) "The number of agents must agree between source vertices, target vertices, and departure time"
    check_overlap_on_vertex(source_vertices, "Invalid source vertices for agents")
    check_overlap_on_vertex(target_vertices, "Invalid target vertices for agents")
    @assert 1 <= ndims(vertex_cost) <= 2 "Vertex cost can only be 2 dimensional (agent, vertex) or 1 (vertex), but get $(ndims(vertex_cost))-dimensions"
    @assert 2 <= ndims(edge_cost) <= 3 "Edge cost can only be 3 dimensional (agent, vertex, vertex) or 2 (vertex, vertex), but get $(ndims(edge_cost))-dimensions"
    @assert all(departure_time .>= zero(eltype(departure_time))) "Departure time must be non-negative integer"

    if is_directed(network)
        edge_tuples = [(src(ed), dst(ed)) for ed in edges(network)]
    else
        edge_tuples = reduce(
            vcat, [[(src(ed), dst(ed)), (dst(ed), src(ed))] for ed in edges(network)]
        )
    end
    n_agents = length(source_vertices)

    all_agents = Base.OneTo(n_agents)

    # Variables
    edge_select_vars = @variable(
        model,
        0 <= x[a=all_agents, e=edge_tuples, t=0:(time_duration - 1)] <= 1;
        integer=integer
    )
    model[(Symbol(edge_var_name))] = edge_select_vars

    vertex_select_vars = @variable(
        model,
        0 <= y[a=all_agents, v=vertices(network), t=0:(time_duration - 1)] <= 1;
        integer=integer
    )
    model[(Symbol(vertex_var_name))] = vertex_select_vars

    ## Constraints
    # Connectivity constraints
    for (agent_id, (agent_source, agent_target)) in
        enumerate(zip(source_vertices, target_vertices))
        agent_departure_time = departure_time[agent_id]

        # XXX: do we need to prohibit actions before departure time?

        # Agents leave from their sources
        @constraint(
            model,
            sum(
                sum(
                    edge_select_vars[agent_id, (agent_source, next_v), t] for
                    next_v in outneighbors(network, agent_source)
                ) - sum(
                    edge_select_vars[agent_id, (prev_v, agent_source), t] for
                    prev_v in inneighbors(network, agent_source)
                ) for t in agent_departure_time:(time_duration - 1)
            ) == 1
        )

        # Agents enter their destination and stays there
        @constraint(
            model,
            sum(
                sum(
                    edge_select_vars[agent_id, (agent_target, next_v), t] for
                    next_v in outneighbors(network, agent_target)
                ) + sum(
                    edge_select_vars[agent_id, (prev_v, agent_target), t] for
                    prev_v in inneighbors(network, agent_target)
                ) for t in agent_departure_time:(time_duration - 1)
            ) == 1
        )

        if vertex_binding
            # Agents can only leave the vertex if it enters the previous time slot
            for v in vertices(network), t in 1:(time_duration - 1)
                @constraint(
                    model,
                    sum(
                        edge_select_vars[agent_id, (prev_v, v), t - 1] for
                        prev_v in inneighbors(network, v)
                    ) == sum(
                        edge_select_vars[agent_id, (v, next_v), t] for
                        next_v in outneighbors(network, v)
                    )
                )

                # Agent must visit a vertex if it travels via any inbounding edge
                @constraint(
                    model,
                    vertex_select_vars[agent_id, v, t] == sum(
                        edge_select_vars[agent_id, (prev_v, v), t - 1] for
                        prev_v in inneighbors(network, v)
                    )
                )
            end

            # Agent must at most present at one vertex
            for t in agent_departure_time:(time_duration - 1)
                @constraint(
                    model,
                    sum(vertex_select_vars[agent_id, v, t] for v in vertices(network)) <= 1
                )
            end

        else
            for v in vertices(network), t in agent_departure_time:(time_duration - 2)
                # Agents can only leave the vertex if it is already there
                @constraint(
                    model,
                    vertex_select_vars[agent_id, v, t] + sum(
                        edge_select_vars[agent_id, (prev_v, v), t] for
                        prev_v in inneighbors(network, v)
                    ) ==
                        vertex_select_vars[agent_id, v, t + 1] + sum(
                        edge_select_vars[agent_id, (v, next_v), t + 1] for
                        next_v in outneighbors(network, v)
                    )
                )

                # Agent can only choose to either stay at the vertex or go to the next vertex
                @constraint(
                    model,
                    vertex_select_vars[agent_id, v, t] + sum(
                        edge_select_vars[agent_id, (v, next_v), t] for
                        next_v in outneighbors(network, v)
                    ) <= 1
                )
            end

            # Agent must at most present at a vertex or an edge throughout its lifespan
            for t in agent_departure_time:(time_duration - 1)
                @constraint(
                    model,
                    sum(vertex_select_vars[agent_id, v, t] for v in vertices(network)) +
                    sum(edge_select_vars[agent_id, ed, t] for ed in edge_tuples) == 1
                )
            end

            # Agent must pay the cost of vertex if pass through it
            if vertex_visit in (:auto, :yes)
                for v in vertices(network), t in agent_departure_time:(time_duration - 2)
                    vcost = if ndims(vertex_cost) == 2
                        vertex_cost[agent_id, v]
                    else
                        vertex_cost[v]
                    end
                    if vertex_visit == :yes || vcost > zero(eltype(vertex_cost))
                        @constraint(
                            model,
                            vertex_select_vars[agent_id, v, t + 1] >= sum(
                                edge_select_vars[agent_id, (prev_v, v), t] for
                                prev_v in inneighbors(v)
                            )
                        )
                    end
                end
            end
        end
    end

    # Conflict Constraints
    # vertex conflict: at any given time, no more than 1 agent can occupy a vertex
    for v in vertices(network), t in 0:(time_duration - 1)
        @constraint(
            model,
            sum(
                vertex_select_vars[agent_id, v, t] + sum(
                    edge_select_vars[agent_id, (prev_v, v), t] for
                    prev_v in inneighbors(network, v)
                ) for agent_id in 1:n_agents
            ) <= 1
        )
    end
    # edge conflict: at any given time, no more than 1 agent can occupy an edge
    for ed in edge_tuples, t in 0:(time_duration - 1)
        @constraint(
            model, sum(edge_select_vars[agent_id, ed, t] for agent_id in 1:n_agents) <= 1
        )
    end

    # Swapping conflict
    for (u, v) in edge_tuples
        if (v, u) ∉ edge_tuples
            continue
        end
        for t in 0:(time_duration - 1)
            @constraint(
                model,
                sum(
                    edge_select_vars[agent_id, (u, v), t] +
                    edge_select_vars[agent_id, (v, u), t] for agent_id in 1:n_agents
                ) <= 1
            )
        end
    end

    # Objective
    if ndims(edge_cost) == 3
        edge_objective = sum(
            edge_cost[agent_id, u, v] * edge_select_vars[agent_id, (u, v), t] for
            (u, v) in edge_tuples, agent_id in 1:n_agents, t in 0:(time_duration - 1)
        )
    else
        edge_objective = sum(
            edge_cost[u, v] * edge_select_vars[agent_id, (u, v), t] for
            (u, v) in edge_tuples, agent_id in 1:n_agents, t in 0:(time_duration - 1)
        )
    end

    if ndims(vertex_cost) == 2
        vertex_objective = sum(
            vertex_cost[agent_id, v] * vertex_select_vars[agent_id, v, t] for
            v in vertices(network), agent_id in 1:n_agents, t in 0:(time_duration - 1)
        )
    else
        vertex_objective = sum(
            vertex_cost[v] * vertex_select_vars[agent_id, v, t] for v in vertices(network),
            agent_id in 1:n_agents, t in 0:(time_duration - 1)
        )
    end

    @objective(model, Min, edge_objective + vertex_objective)
end

"""
    mapf_discrete_time(
        network, source_vertices, target_vertices, vertex_cost, edge_cost, departure_time;
        time_duration, integer, vertex_binding, optimizer, silent
    )

Compute the MAPH problem in discrete time from a set of source vertices to target vertices within a time span.
Traversal of each vertex and edge comes with a cost.
Each agent has a unique departure time from its source vertex.
Returns the selected vertices and edges for each agent

# Arguments

- `network::AbstractGraph`: a directed graph representing the map
- `source_vertices::Vector{Int}`: an array of vertices indicating each agent's source vertex (the vertex an agent starts travel from)
- `target_vertices::Vector{Int}`: an array of vertices indicating each agent's target vertex (the vertex an agent end its travel at)
- `vertex_cost::Array{<:Real}`: costs for staying at each vertex. dimension = ([agent,] vertex)
- `edge_cost::Array{<:Real}`: costs for crossing each edge. dimension = ([agent,] vertex, vertex), we use (from_vertex, to_vertex) to indicate an edge
- `departure_time::Vector{Int}`:  (default is `zeros(Int, length(source_vertices))`)

# Keyword arguments

- `time_duration::Int`: the maximum time the system runs (default is number of edges of `network`)
- `integer::Bool`: whether the path should be integer-valued or real-valued (default is `true`)
- `vertex_binding::Bool`: whether required to stay and pay the cost for every traversal of vertex
- `optimizer`: JuMP-compatible solver (default is `HiGHS.Optimizer`)
- `silent::Bool`: turn of printing of model status printing

"""
function mapf_discrete_time(
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    # dim: [agent,] vertex 
    vertex_cost::AbstractArray,
    # dim: [agent,] from_vertex, to_vertex
    edge_cost::AbstractArray,
    departure_time::Vector{Int}=zeros(Int, length(source_vertices));
    time_duration::Int=ne(network),
    integer::Bool=true,
    vertex_binding::Bool=false,
    vertex_visit::Symbol=:auto, # `:auto` needs to wait if vertex cost > 0, `:yes` means always needs to wait, `:no` means not enforced
    optimizer=HiGHS.Optimizer,
    silent::Bool=true,
)
    model = Model(optimizer)
    if silent
        set_silent(model)
    end

    mapf_discrete_time!(
        model,
        network,
        source_vertices,
        target_vertices,
        vertex_cost,
        edge_cost,
        departure_time;
        time_duration,
        vertex_var_name=:vertex,
        edge_var_name=:edge,
        vertex_binding=vertex_binding,
        vertex_visit=vertex_visit,
        integer=integer,
    )
    optimize!(model)
    @assert termination_status(model) == OPTIMAL

    edge_selection_vars = value.(model[:edge])
    vertex_selection_vars = value.(model[:vertex])

    # parse vertices
    agents, selection, timestamps = axes(vertex_selection_vars)
    valid_vertices = [
        [
            (t, v) for
            v in selection, t in timestamps if vertex_selection_vars[agent_id, v, t] > 0.5
        ] for agent_id in agents
    ]

    # parse edges
    agents, selection, timestamps = axes(edge_selection_vars)
    valid_edges = [
        [
            (t, ed) for
            ed in selection, t in timestamps if edge_selection_vars[agent_id, ed, t] > 0.5
        ] for agent_id in agents
    ]

    return valid_vertices, valid_edges
end
