
"""
    add_continuous_connectivity_constraints!(
        model,
        network, source_vertices, target_vertices,
        vertex_select_vars, edge_select_vars
    )
"""
function add_continuous_connectivity_constraints!(
    model::Model,
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    vertex_select_vars,
    edge_select_vars,
)
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
end

"""
    add_continuous_timing_constraints!(
        model,
        network, source_vertices, edge_tuples,
        vertex_wait_time, edge_wait_time, departure_time,
        vertex_select_vars, edge_select_vars, vertex_arrival_time, edge_arrival_time;
        big_M
    )
Add timing constraints so that the proper arrival time of agents to each vertex and edge is computed
"""
function add_continuous_timing_constraints!(
    model::Model,
    network::AbstractGraph,
    source_vertices::Vector{Int},
    edge_tuples::Vector{Tuple{Int,Int}},
    vertex_wait_time::AbstractArray,
    edge_wait_time::AbstractArray,
    departure_time::Vector{T},
    vertex_select_vars,
    edge_select_vars,
    vertex_arrival_time,
    edge_arrival_time;
    big_M::Real=100.0,
) where {T<:Real}
    n_agents = length(source_vertices)

    for agent_id in 1:n_agents
        agent_source = source_vertices[agent_id]

        # Base case, the starting time of each agent from source vertices
        @constraint(
            model, vertex_arrival_time[agent_id, agent_source] == departure_time[agent_id]
        )

        # Iterative case, all edges arrival time waits their previous arrival of source vertices
        for v in vertices(network)
            wait_time = if ndims(vertex_wait_time) == 2
                vertex_wait_time[agent_id, v]
            else
                vertex_wait_time[v]
            end

            for next_v in outneighbors(network, v)
                @constraint(
                    model,
                    edge_arrival_time[agent_id, (v, next_v)] >=
                        vertex_arrival_time[agent_id, v] +
                    vertex_select_vars[agent_id, v] * (wait_time + big_M) - big_M
                )
            end
        end

        # all vertex arrival time is larger than edge arrival + travel time if it travels through the edge
        for (u, v) in edge_tuples
            wait_time = if ndims(edge_wait_time) == 3
                edge_wait_time[agent_id, u, v]
            else
                edge_wait_time[u, v]
            end

            @constraint(
                model,
                vertex_arrival_time[agent_id, v] >=
                    edge_arrival_time[agent_id, (u, v)] +
                edge_select_vars[agent_id, (u, v)] * (wait_time + big_M) - big_M
            )
        end
    end
end

"""
    add_continuous_conflict_constraints!(
        model,
        network, source_vertices, edge_tuples,
        vertex_arrival_time, edge_arrival_time,
        vertex_prior_margin, edge_prior_margin, vertex_post_margin, edge_post_margin;
        integer, merge_margin, big_M
    )
Add vertex/edge/swap constraints to model
"""
function add_continuous_conflict_constraints!(
    model::Model,
    network::AbstractGraph,
    source_vertices::Vector{Int},
    edge_tuples::Vector{Tuple{Int,Int}},
    vertex_arrival_time,
    edge_arrival_time,
    vertex_prior_margin::Array{T}=zeros(nv(network)),
    edge_prior_margin::Array{T}=zeros((nv(network), nv(network))),
    vertex_post_margin::Array{T}=zeros(nv(network)),
    edge_post_margin::Array{T}=zeros((nv(network), nv(network)));
    integer::Bool=true,
    merge_margin::Bool=false,
    big_M::Real=100.0,
    swap_constraint::Bool=true,
) where {T<:Real}
    n_agents = length(source_vertices)
    both_way_edge_tuples = [
        (u, v) for (u, v) in edge_tuples if u <= v && (v, u) in edge_tuples
    ]

    vertex_entry_sequence = @variable(
        model,
        0 <= v_seq[a1=1:n_agents, a2=1:n_agents, v=vertices(network); a2 > a1] <= 1;
        integer=integer
    )
    edge_entry_sequence = @variable(
        model,
        0 <= e_seq[a1=1:n_agents, a2=1:n_agents, ed=edge_tuples; a2 > a1] <= 1;
        integer=integer
    )
    swapping_var = @variable(
        model,
        0 <= sw[a1=1:n_agents, a2=1:n_agents, ed=both_way_edge_tuples; a2 > a1] <= 1;
        integer=integer
    )
    for agent_i in 1:n_agents, agent_j in (agent_i + 1):n_agents
        # Vertex Conflict
        for v in vertices(network)
            for next_v in outneighbors(network, v)
                @constraint(
                    model,
                    vertex_arrival_time[agent_i, v] >=
                        edge_arrival_time[agent_j, (v, next_v)] -
                    big_M * vertex_entry_sequence[agent_i, agent_j, v]
                )
                @constraint(
                    model,
                    vertex_arrival_time[agent_j, v] >=
                        edge_arrival_time[agent_i, (v, next_v)] -
                    big_M * (1 - vertex_entry_sequence[agent_i, agent_j, v])
                )
            end
        end

        # Edge Conflict
        for (u, v) in edge_tuples
            if agent_i >= agent_j
                continue
            end

            @constraint(
                model,
                edge_arrival_time[agent_i, (u, v)] >=
                    vertex_arrival_time[agent_j, v] -
                big_M * edge_entry_sequence[agent_i, agent_j, (u, v)]
            )
            @constraint(
                model,
                edge_arrival_time[agent_j, (u, v)] >=
                    vertex_arrival_time[agent_i, v] -
                big_M * (1 - edge_entry_sequence[agent_i, agent_j, (u, v)])
            )
        end

        if swap_constraint
            # Swapping Conflict
            for (u, v) in both_way_edge_tuples
                if agent_i >= agent_j
                    continue
                end

                @constraint(
                    model,
                    edge_arrival_time[agent_i, (u, v)] >=
                        vertex_arrival_time[agent_j, u] -
                    big_M * swapping_var[agent_i, agent_j, (u, v)]
                )
                @constraint(
                    model,
                    edge_arrival_time[agent_j, (v, u)] >=
                        vertex_arrival_time[agent_i, v] -
                    big_M * (1 - swapping_var[agent_i, agent_j, (u, v)])
                )
            end
        end
    end
end

"""
    mapf_continuous_time!(
        model,
        network, source_vertices, target_vertices,
        vertex_wait_time, edge_wait_time,
        vertex_cost, edge_cost,
        departure_time;
        vertex_var_name, edge_var_name,
        vertex_arrival_time_var_name, edge_arrival_time_var_name
        integer, big_M
    )

Modify a JuMP model by adding the variable, constraints and objective to compute continuous-time MAPH problem

# Arguments

- `model::Model`: `JuMP model` to be modified (adding variable, constraints, and objective)
- `network::AbstractGraph`: a directed graph representing the map
- `source_vertices::Vector{Int}`: an array of vertices indicating each agent's source vertex (the vertex an agent starts travel from)
- `target_vertices::Vector{Int}`: an array of vertices indicating each agent's target vertex (the vertex an agent end its travel at)
- `vertex_wait_time::Array{<:Real}`: minimum staying time for agent at each vertex. dimension = ([agent,] vertex)
- `edge_wait_time::Array{<:Real}`: minimum travel time for agent on each edge. dimension = ([agent,] from_vertex, to_vertex)
- `vertex_cost::Array{<:Real}`: costs for staying at each vertex. dimension = ([agent,] vertex)
- `edge_cost::Array{<:Real}`: costs for crossing each edge. dimension = ([agent,] from_vertex, to_vertex)
- `departure_time::Vector{Float64}`: departure time for each agent at their source vertex

# Keyword arguments

- `vertex_var_name`: name of vertex selection variables, will be intepreted as a symbol
- `edge_var_name`: name of edge selection variables, will be intepreted as a symbol
- `vertex_arrival_time_var_name`: name of variable for arrival time of agent `i` at vertex `v`
- `edge_arrival_time_var_name`: name of veriable for arrival time of agent `i` at edge `(u, v)`
- `integer::Bool`: whether to apply integer programming, apply linear relaxation otherwise
- `big_M::{<:Real}: the big constant for if-else statements as linear constraint, should be larger than any time measure`

"""
function mapf_continuous_time!(
    model::Model,
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    # dim: [agent,] vertex
    vertex_wait_time::AbstractArray,
    # dim: [agent,] from_vertex, to_vertex
    edge_wait_time::AbstractArray,
    vertex_cost::AbstractArray=vertex_wait_time,
    edge_cost::AbstractArray=edge_wait_time,
    departure_time::Vector{T}=zeros(length(source_vertices));
    vertex_var_name=:vertex,
    edge_var_name=:edge,
    vertex_arrival_time_var_name=:vertex_arrival_time,
    edge_arrival_time_var_name=:edge_arrival_time,
    integer::Bool=true,
    big_M::Real=100.0,
    swap_constraint::Bool=true,
) where {T<:Real}
    @assert length(source_vertices) == length(target_vertices) "The number of source vertices does not match the number of target vertices"
    check_overlap_on_vertex(source_vertices, "Invalid source vertices for agents")
    check_overlap_on_vertex(target_vertices, "Invalid target vertices for agents")
    @assert 1 <= ndims(vertex_cost) <= 2 "Vertex cost can only be 2 dimensional (agent, vertex) or 1 (vertex), but get $(ndims(vertex_cost))-dimensions"
    @assert 2 <= ndims(edge_cost) <= 3 "Edge cost can only be 3 dimensional (agent, vertex, vertex) or 2 (vertex, vertex), but get $(ndims(edge_cost))-dimensions"
    @assert all(departure_time .>= zero(eltype(departure_time))) "Departure time must be non-negative"

    if is_directed(network)
        edge_tuples = [(src(ed), dst(ed)) for ed in edges(network)]
    else
        edge_tuples = reduce(
            vcat, [[(src(ed), dst(ed)), (dst(ed), src(ed))] for ed in edges(network)]
        )
    end
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

    vertex_arrival_time = @variable(model, v_arr[a=1:n_agents, v=vertices(network)] >= 0)
    model[Symbol(vertex_arrival_time_var_name)] = vertex_arrival_time

    edge_arrival_time = @variable(model, e_arr[a=1:n_agents, ed=edge_tuples] >= 0)
    model[Symbol(edge_arrival_time_var_name)] = edge_arrival_time

    ## Constraints
    # Connectivity constraints
    add_continuous_connectivity_constraints!(
        model,
        network,
        source_vertices,
        target_vertices,
        vertex_select_vars,
        edge_select_vars,
    )

    # Timing constraints
    add_continuous_timing_constraints!(
        model,
        network,
        source_vertices,
        edge_tuples,
        vertex_wait_time,
        edge_wait_time,
        departure_time,
        vertex_select_vars,
        edge_select_vars,
        vertex_arrival_time,
        edge_arrival_time;
        big_M=big_M,
    )

    ## Conflicts
    add_continuous_conflict_constraints!(
        model,
        network,
        source_vertices,
        edge_tuples,
        vertex_arrival_time,
        edge_arrival_time;
        integer=integer,
        big_M=big_M,
        swap_constraint=swap_constraint,
    )

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

    vertex_time_objective = sum(vertex_arrival_time)
    edge_time_objective = sum(edge_arrival_time)

    @objective(
        model,
        Min,
        edge_objective + vertex_objective + vertex_time_objective + edge_time_objective
    )

    return model
end

"""
    mapf_continuous_time(
        network, source_vertices, target_vertices, vertex_cost, edge_cost, departure_time;
        integer, optimizer, silent, big_M
    )

Compute the MAPH problem in continuous time from a set of source vertices to target vertices.
Traversal of each vertex and edge comes with a cost.
Returns the selected vertices and edges for each agent

# Arguments

- `network::AbstractGraph`: a directed graph representing the map
- `source_vertices::Vector{Int}`: an array of vertices indicating each agent's source vertex (the vertex an agent starts travel from)
- `target_vertices::Vector{Int}`: an array of vertices indicating each agent's target vertex (the vertex an agent end its travel at)
- `vertex_wait_time::Array{<:Real}`: waiting time for each agent to stay at a specific vertex. dimension = ([agent,] vertex)
- `edge_wait_time::Array{<:Real}`: waiting time for each agent to travel on a specific edge. dimension = ([agent,] vertex, vertex), we use (from_vertex, to_vertex) to indicate an edge
- `vertex_cost::Array{<:Real}`: costs for staying at each vertex. dimension = ([agent,] vertex). by default `= vertex_wait_time`
- `edge_cost::Array{<:Real}`: costs for crossing each edge. dimension = ([agent,] vertex, vertex), we use (from_vertex, to_vertex) to indicate an edge. by default `= edge_wait_time`
- `departure_time::Vector{Float64}`: departure time of each agent (default is `zeros(Float64, length(source_vertices))`)

# Keyword arguments

- `integer::Bool`: whether the path should be integer-valued or real-valued (default is `true`)
- `optimizer`: JuMP-compatible solver (default is `HiGHS.Optimizer`)
- `silent::Bool`: turn of printing of model status printing
- `big_M::{<:Real}: the big constant for if-else statements as linear constraint, should be larger than any time measure`

"""
function mapf_continuous_time(
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    # dim: [agent,] vertex 
    vertex_wait_time::AbstractArray,
    # dim: [agent,] from_vertex, to_vertex
    edge_wait_time::AbstractArray,
    vertex_cost::AbstractArray=vertex_wait_time,
    edge_cost::AbstractArray=edge_wait_time,
    departure_time::Vector{T}=zeros(length(source_vertices));
    integer::Bool=true,
    optimizer=HiGHS.Optimizer,
    silent::Bool=true,
    big_M::Real=100.0,
    swap_constraint::Bool=true,
) where {T<:Real}
    model = Model(optimizer)
    if silent
        set_silent(model)
    end

    mapf_continuous_time!(
        model,
        network,
        source_vertices,
        target_vertices,
        vertex_wait_time,
        edge_wait_time,
        vertex_cost,
        edge_cost,
        departure_time;
        vertex_var_name=:vertex_select,
        edge_var_name=:edge_select,
        vertex_arrival_time_var_name=:vertex_time,
        edge_arrival_time_var_name=:edge_time,
        integer=integer,
        big_M=big_M,
        swap_constraint=swap_constraint,
    )

    optimize!(model)
    @assert termination_status(model) == OPTIMAL

    vertex_selection_vars = value.(model[:vertex_select])
    edge_selection_vars = value.(model[:edge_select])

    vertex_arrival_time = value.(model[:vertex_time])
    edge_arrival_time = value.(model[:edge_time])

    # parse vertices
    agents, selection = axes(vertex_selection_vars)
    valid_vertices = [
        [
            (vertex_arrival_time[agent_id, v], v) for
            v in selection if vertex_selection_vars[agent_id, v] > 0.5
        ] for agent_id in agents
    ]
    # sort by time
    for agent_vertices in valid_vertices
        sort!(agent_vertices; by=(x -> first(x)))
    end

    # parse edges
    agents, selection = axes(edge_selection_vars)
    valid_edges = [
        [
            (edge_arrival_time[agent_id, ed], ed) for
            ed in selection if edge_selection_vars[agent_id, ed] > 0.5
        ] for agent_id in agents
    ]
    # sort by time
    for agent_edges in valid_edges
        sort!(agent_edges; by=(x -> first(x)))
    end

    return valid_vertices, valid_edges
end
