"""
    nonzero_minimum(collection)
Get the nonzero minimum value of a collection
"""
function nonzero_minimum(collection)
    @assert !isempty(collection) "Cannot find minimum over an empty collection"
    nonzero_elements = collection[collection .!= zero(eltype(collection))]
    @assert !isempty(nonzero_elements) "No nonzero value in collection"
    return minimum(nonzero_elements)
end

"""
    parse_result(
        model [, vertex_select_name, edge_select_name, vertex_time_name, edge_time_name]
    )
Parse the result of the MILP MAPF problem from a JuMP solver

# Arguments

- `model::Model`: a JuMP model containing the optimized answer
- `vertex_select_name`: name of vertex selection variable, one can access the variables
by `model[vertex_select_name]`. Default value is `:vertex_select`
- `edge_select_name`: name of edge selection variable, one can access the variables
by `model[edge_select_name]`. Default value is `:edge_select`
- `vertex_time_name`: name of vertex arrival time variable, one can access the variables
by `model[vertex_time_name]`. Default value is `:vertex_time`
- `edge_time_name`: name of edge arrival time variable, one can access the variables
by `model[edge_time_name]`. Default value is `:edge_time`
"""
function parse_result(
    model::Model,
    vertex_select_name=:vertex_select,
    edge_select_name=:edge_select,
    vertex_time_name=:vertex_time,
    edge_time_name=:edge_time,
)
    vertex_selection_vars = value.(model[vertex_select_name])
    edge_selection_vars = value.(model[edge_select_name])

    vertex_arrival_time = value.(model[vertex_time_name])
    edge_arrival_time = value.(model[edge_time_name])

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

"""
    right_align_get(data, idx...)
get item with indices align to the end, only supports over-indexing (indices longer than data dimension)

# Arguments

- `data`: container to retrieve data from
- `idx...`: vargs as indices to get data, can be longer than data dimension and will be aligned to the right (end)

"""
function right_align_get(data, idx...)
    data_dim = ndims(data)
    @assert length(idx) >= data_dim
    align_length = min(length(idx), data_dim)

    if length(idx) > data_dim
        aligned_idx = idx[(end - align_length + 1):end]
    else
        aligned_idx = idx
    end
    return data[aligned_idx...]
end

"""
    add_objective!(
        model,
        network, vertex_cost, edge_cost,
        vertex_select_vars, edge_select_vars, vertex_arrival_time, edge_arrival_time;
        with_timing
    )
Add/update objective of the JuMP model.

# Arguments

- `model::Model`: A JuMP model to update objective function
- `network::AbstractGraph`: A graph representing the network where agents travel upon
- `vertex_cost::AbstractArray`: Costs for staying at each vertex. dimension = ([agent,] vertex).
- `edge_cost::AbstractArray`: Costs for traveling through an edge.dimension = ([agent,] from_vertex, to_vertex).
- `vertex_select_vars`: JuMP variables for vertex selection.
- `edge_select_vars`: JuMP variables for edge selection.
- `vertex_arrival_time`: JuMP variables for vertex arrival time.
- `edge_arrival_time`: JuMP variables for edge arrival time.

# Keyword arguments

- `with_timing`: if `true`, minimize the traveling cost + sum of arrival time; if `false`, minimize the traveling cast only

"""
function add_objective!(
    model::Model,
    network::AbstractGraph,
    target_vertices::Vector{Int},
    vertex_cost::AbstractArray,
    edge_cost::AbstractArray,
    vertex_select_vars,
    edge_select_vars,
    vertex_arrival_time=nothing,
    edge_arrival_time=nothing;
    with_timing::Bool=true,
)
    agents, edge_tuples = axes(edge_select_vars)

    edge_objective = AffExpr(0.0)
    for (u, v) in edge_tuples, agent_id in agents
        add_to_expression!(
            edge_objective,
            right_align_get(edge_cost, agent_id, u, v),
            edge_select_vars[agent_id, (u, v)],
        )
    end

    vertex_objective = AffExpr(0.0)
    for v in vertices(network), agent_id in agents
        add_to_expression!(
            vertex_objective,
            right_align_get(vertex_cost, agent_id, v),
            vertex_select_vars[agent_id, v],
        )
    end

    if !with_timing || isnothing(vertex_arrival_time) || isnothing(edge_arrival_time)
        return @objective(model, Min, edge_objective + vertex_objective)
    end

    time_objective = AffExpr(0.0)
    for (agent_id, agent_target) in enumerate(target_vertices)
        add_to_expression!(time_objective, vertex_arrival_time[agent_id, agent_target])
    end

    return @objective(model, Min, edge_objective + vertex_objective + time_objective)
end

"""
    add_vertex_conflict!(
        model,
        network, vertex, agent1, agent2,
        vertex_arrival_time, edge_arrival_time, time_horizon;
        binary
    )
Add a vertex conflict constraint between `agent1` and `agent2` on `vertex` to the model

# Arguments

- `model::Model`: JuMP model to be modified
- `network::AbstractGraph`: network where agents travel upon
- `vertex::Int`: vertex to avoid conflict
- `agent1::Int`: first involved agent to avoid conflict with the other agent on `vertex`
- `agent2::Int`: second involved agent to avoid conflict with the other agent on `vertex`
- `vertex_arrival_time`: JuMP variable indicating arrival time on vertices
- `edge_arrival_time`: JuMP variable indicating arrival time on edges
- `time_horizon::Float64`: maximum planning time of the scheduling problem. Value too small may cause
incorrect result while too large can slow the program down. By default `100.0`

# Keyword arguments

- `binary::Bool`: whether to use binary variable {0, 1} or its relaxation [0, 1]

"""
function add_vertex_conflict!(
    model::Model,
    network::AbstractGraph,
    vertex::Int,
    agent1::Int,
    agent2::Int,
    vertex_arrival_time,
    edge_arrival_time,
    edge_select_vars,
    time_horizon=100.0;
    binary::Bool=true,
    heuristic_conflict::Bool=false,
    safety_gap=1e-6,
)
    # Obtain the end time of vertex occupancy by the arrival time of outgoing edges
    agent1_end_time_candidates = [
        edge_arrival_time[agent1, (vertex, next_v)] for
        next_v in outneighbors(network, vertex) if
        value(edge_select_vars[agent1, (vertex, next_v)]) >= 0.9
    ]
    agent2_end_time_candidates = [
        edge_arrival_time[agent2, (vertex, next_v)] for
        next_v in outneighbors(network, vertex) if
        value(edge_select_vars[agent2, (vertex, next_v)]) >= 0.9
    ]

    # if not found (no outgoing edge is selected), it means the end time is Inf
    if isempty(agent1_end_time_candidates)
        @constraint(
            model,
            vertex_arrival_time[agent1, vertex] >=
                first(agent2_end_time_candidates) + safety_gap
        )
        return nothing
    end
    agent1_end_time = first(agent1_end_time_candidates)

    if isempty(agent2_end_time_candidates)
        @constraint(
            model, vertex_arrival_time[agent2, vertex] >= agent1_end_time + safety_gap
        )
        return nothing
    end
    agent2_end_time = first(agent2_end_time_candidates)

    if heuristic_conflict
        if abs(value(vertex_arrival_time[agent1, vertex]) - value(agent2_end_time)) <=
            abs(value(vertex_arrival_time[agent2, vertex]) - value(agent1_end_time))
            @constraint(model, vertex_arrival_time[agent1, vertex] >= agent2_end_time)
        else
            @constraint(model, vertex_arrival_time[agent2, vertex] >= agent1_end_time)
        end
    else
        entry_sequence = @variable(model, lower_bound = 0, upper_bound = 1, binary = binary)
        @constraint(
            model,
            vertex_arrival_time[agent1, vertex] >=
                agent2_end_time + safety_gap - time_horizon * entry_sequence
        )
        @constraint(
            model,
            vertex_arrival_time[agent2, vertex] >=
                agent1_end_time + safety_gap - time_horizon * (1 - entry_sequence)
        )
    end
end

"""
    add_edge_conflict!(
        model,
        network, edge, agent1, agent2, is_swap,
        vertex_arrival_time, edge_arrival_time, time_horizon;
        binary
    )
Add an edge or swapping conflict constraint between `agent1` and `agent2` on `edge` to the model

# Arguments

- `model::Model`: JuMP model to be modified
- `network::AbstractGraph`: network where agents travel upon
- `edge::Tuple{Int, Int}`: edge to avoid conflict
- `agent1::Int`: first involved agent to avoid conflict with the other agent on `edge`
- `agent2::Int`: second involved agent to avoid conflict with the other agent on `edge`
- `vertex_arrival_time`: JuMP variable indicating arrival time on vertices
- `edge_arrival_time`: JuMP variable indicating arrival time on edges
- `time_horizon::Float64`: maximum planning time of the scheduling problem. Value too small may cause
incorrect result while too large can slow the program down. By default `100.0`

# Keyword arguments

- `binary::Bool`: whether to use binary variable {0, 1} or its relaxation [0, 1]

"""
function add_edge_conflict!(
    model::Model,
    edge::Tuple{Int,Int},
    agent1::Int,
    agent2::Int,
    is_swap::Bool,
    vertex_arrival_time,
    edge_arrival_time;
    time_horizon::Float64=100.0,
    binary::Bool=true,
    heuristic_conflict::Bool=false,
    safety_gap=1e-4,
)
    ed2 = is_swap ? reverse(edge) : edge

    if heuristic_conflict
        if abs(
            value(edge_arrival_time[agent1, edge]) -
            value(vertex_arrival_time[agent2, ed2[2]]),
        ) <= abs(
            value(edge_arrival_time[agent2, ed2]) -
            value(vertex_arrival_time[agent1, edge[2]]),
        )
            @constraint(
                model,
                edge_arrival_time[agent1, edge] >= vertex_arrival_time[agent2, ed2[2]]
            )
        else
            @constraint(
                model,
                edge_arrival_time[agent2, ed2] >= vertex_arrival_time[agent1, edge[2]]
            )
        end
    else
        entry_sequence = @variable(model, lower_bound = 0, upper_bound = 1, binary = binary)

        @constraint(
            model,
            edge_arrival_time[agent1, edge] >=
                vertex_arrival_time[agent2, ed2[2]] + safety_gap -
            time_horizon * entry_sequence
        )
        @constraint(
            model,
            edge_arrival_time[agent2, ed2] >=
                vertex_arrival_time[agent1, edge[2]] + safety_gap -
            time_horizon * (1 - entry_sequence)
        )
    end
end

"""
    is_path_overlap(
        model [, vertex_select_name, edge_select_name]
    )
Return if any two agents' paths overlap each other.

# Arguments

- `model::Model`: JuMP model storing the result
- `vertex_select_name`: name of vertex selection variable, one can access the variables
by `model[vertex_select_name]`. Default value is `:vertex_select`
- `edge_select_name`: name of edge selection variable, one can access the variables
by `model[edge_select_name]`. Default value is `:edge_select`

# Keyword arguments
- `swap_constraint::Bool`: whether to apply swap constraint, i.e. edge (u, v) and (v, u)
cannot be crossed at the same time
"""
function is_path_overlap(
    model::Model,
    vertex_select_name::Symbol=:vertex_select,
    edge_select_name::Symbol=:edge_select;
    swap_constraint::Bool=true,
)::Bool
    vertex_select_vars = value.(model[vertex_select_name])
    agents, traveled_vertices = axes(vertex_select_vars)
    # vertex_occupancy[v] stores the indices of any agent occupies v, <= 0 means no agent occupy yet
    vertex_occupancy = Set{Int}()
    for agent_id in agents, v in traveled_vertices
        if vertex_select_vars[agent_id, v] > 0.5
            if v in vertex_occupancy
                return true
            end
            push!(vertex_occupancy, v)
        end
    end

    edge_select_vars = value.(model[edge_select_name])
    agents, traveled_edges = axes(edge_select_vars)
    # edge_occupancy[ed] stores the indices of any agent occupies ed, <= 0 means no agent occupy yet
    edge_occupancy = Set{Tuple{Int,Int}}()
    for agent_id in agents, ed in traveled_edges
        if swap_constraint && ed[1] > ed[2]
            reverse!(ed)
        end
        if edge_select_vars[agent_id, ed] > 0.5
            if ed in edge_occupancy
                return true
            end
            push!(edge_occupancy, ed)
        end
    end

    return false
end

"""
    parallel_shortest_path_result(
        source_vertices, departure_time,
        vertex_wait_time, edge_wait_time,
        vertex_select_vars, edge_select_vars
    )
Calculate the timed result for parallel shortest path solution, returns the vertex and edge arrival timing

# Arguments

- `source_vertices::Vector{Int}`: an array of vertices indicating each agent's source vertex (the vertex an agent starts travel from)
- `departure_time::Vector{Float64}`: departure time for each agent at their source vertex, by default all zeros
- `vertex_wait_time::AbstractArray`: minimum staying time for agent at each vertex. dimension = ([agent,] vertex)
- `edge_wait_time::AbstractArray`: minimum travel time for agent on each edge. dimension = ([agent,] from_vertex, to_vertex)
- `vertex_select_vars`: JuMP variables for vertex selection.
- `edge_select_vars`: JuMP variables for edge selection.

"""
function parallel_shortest_path_result(
    source_vertices::Vector{Int},
    departure_time::Vector{Float64},
    vertex_wait_time::AbstractArray,
    edge_wait_time::AbstractArray,
    vertex_select_vars,
    edge_select_vars,
)
    # Choose selected vertices
    vertex_select_values = value.(vertex_select_vars)
    agents, traveled_vertices = axes(vertex_select_values)
    valid_vertices = [
        [v for v in traveled_vertices if vertex_select_values[agent_id, v] > 0.5] for
        agent_id in agents
    ]

    # Choose selected edges
    edge_select_values = value.(edge_select_vars)
    agents, traveled_edges = axes(edge_select_values)
    valid_edges = [
        [ed for ed in traveled_edges if edge_select_values[agent_id, ed] > 0.5] for
        agent_id in agents
    ]

    # Timing record to be returned
    vertex_timing = [similar(vs, Tuple{Float64,Int}) for vs in valid_vertices]
    edge_timing = [similar(eds, Tuple{Float64,Tuple{Int,Int}}) for eds in valid_edges]

    for agent_id in agents
        prev_v = source_vertices[agent_id]
        event_time = departure_time[agent_id]
        vertex_timing[agent_id][begin] = (event_time, prev_v)

        for step in 1:length(valid_edges[agent_id])
            # edge
            edge = first(edge for edge in valid_edges[agent_id] if edge[begin] == prev_v)
            event_time += right_align_get(edge_wait_time, agent_id, edge...)
            edge_timing[agent_id][step] = (event_time, edge)

            # vertex
            prev_v = edge[end]
            event_time += right_align_get(vertex_wait_time, agent_id, prev_v)
            vertex_timing[agent_id][step + 1] = (event_time, prev_v)
        end
    end

    return vertex_timing, edge_timing
end

"""
    mapf_continuous_time_dynamic_conflict(
        network, source_vertices, target_vertices,
        vertex_wait_time, edge_wait_time, vertex_cost, edge_cost,
        departure_time;
        optimizer
        is_binary, silent, swap_constraint,
        time_horizon, timeout
    )
Compute continuous-time MPAF where the conflict constraints are added dynamically

# Arguments

- `network::AbstractGraph`: network where agents travel upon
- `source_vertices::Vector{Int}`: an array of vertices indicating each agent's source vertex (the vertex an agent starts travel from)
- `target_vertices::Vector{Int}`: an array of vertices indicating each agent's target vertex (the vertex an agent end its travel at)
- `vertex_wait_time::AbstractArray`: minimum staying time for agent at each vertex. dimension = ([agent,] vertex)
- `edge_wait_time::AbstractArray`: minimum travel time for agent on each edge. dimension = ([agent,] from_vertex, to_vertex)
- `vertex_cost::AbstractArray`: costs for staying at each vertex. dimension = ([agent,] vertex)
- `edge_cost::AbstractArray`: costs for crossing each edge. dimension = ([agent,] from_vertex, to_vertex)
- `departure_time::Vector{Float64}`: departure time for each agent at their source vertex, by default all zeros

# Keyword arguments

- `optimizer`: optimizer used in JuMP, by default HiGHS.Optimizer
- `is_binary::Bool`: whether to use binary variable {0, 1} or its relaxation [0, 1]
- `silent::Bool`: turn of printing of model status printing
- `swap_constraint::Bool`: whether to apply swapping constraints
- `time_horizon::Float64`: maximum planning time of the scheduling problem. Value too small may cause
incorrect result while too large can slow the program down. By default `100.0`
- `timeout::Float64`: terminate the optimizer after timeout (unit in seconds)

"""
function mapf_continuous_time_dynamic_conflict(
    network::AbstractGraph,
    source_vertices::Vector{Int},
    target_vertices::Vector{Int},
    # dim: [agent,] vertex 
    vertex_wait_time::AbstractArray,
    # dim: [agent,] from_vertex, to_vertex
    edge_wait_time::AbstractArray,
    vertex_cost::AbstractArray=vertex_wait_time,
    edge_cost::AbstractArray=edge_wait_time,
    departure_time::Vector{Float64}=zeros(Float64, length(source_vertices));
    optimizer=HiGHS.Optimizer,
    is_binary::Bool=true,
    silent::Bool=true,
    swap_constraint::Bool=true,
    time_horizon::Float64=100.0,
    timeout::Float64=-1.0,
    heuristic_conflict::Bool=false,
    epsilon=1e-4 * min(nonzero_minimum(vertex_wait_time), nonzero_minimum(edge_wait_time)),
)
    @assert length(source_vertices) == length(target_vertices) "The number of source vertices does not match the number of target vertices"
    check_overlap_on_vertex(source_vertices, "Invalid source vertices for agents")
    check_overlap_on_vertex(target_vertices, "Invalid target vertices for agents")
    @assert 1 <= ndims(vertex_cost) <= 2 "Vertex cost can only be 2 dimensional (agent, vertex) or 1 (vertex), but get $(ndims(vertex_cost))-dimensions"
    @assert 2 <= ndims(edge_cost) <= 3 "Edge cost can only be 3 dimensional (agent, vertex, vertex) or 2 (vertex, vertex), but get $(ndims(edge_cost))-dimensions"
    @assert all(departure_time .>= zero(eltype(departure_time))) "Departure time must be non-negative"

    model = Model(optimizer)
    if silent
        set_silent(model)
    end
    if timeout > 0.0
        set_time_limit_sec(model, timeout)
    end

    edge_tuples = [(src(ed), dst(ed)) for ed in edges(network)]
    # add edges (v, u) to edge tuples and if edges are defined as (u, v) in an undirected graph
    if !is_directed(network)
        edge_tuples = reduce(
            vcat, [edge_tuples, [(dst(ed), src(ed)) for ed in edges(network)]]
        )
    end
    agents = axes(source_vertices)[begin]

    # Variables
    vertex_select_vars = @variable(
        model, 0 <= y[a=agents, v=vertices(network)] <= 1, binary = is_binary
    )
    model[:vertex_select] = vertex_select_vars

    edge_select_vars = @variable(
        model, 0 <= x[a=agents, e=edge_tuples] <= 1, binary = is_binary
    )
    model[:edge_select] = edge_select_vars

    # Solve parallel shortest path first
    add_continuous_connectivity_constraints!(
        model,
        network,
        source_vertices,
        target_vertices,
        vertex_select_vars,
        edge_select_vars,
    )

    # Apply objective without consider timing
    add_objective!(
        model,
        network,
        target_vertices,
        vertex_cost,
        edge_cost,
        vertex_select_vars,
        edge_select_vars;
        with_timing=false,
    )

    optimize!(model)
    @assert termination_status(model) == OPTIMAL "Parallel shortest paths cannot find solution"
    all_vars = all_variables(model)
    current_solution = value.(all_vars)

    if !is_path_overlap(model; swap_constraint=swap_constraint)
        valid_vertices, valid_edges = parallel_shortest_path_result(
            source_vertices,
            departure_time,
            vertex_wait_time,
            edge_wait_time,
            vertex_select_vars,
            edge_select_vars,
        )

        return valid_vertices, valid_edges, objective_value(model)
    end

    vertex_arrival_time = @variable(model, v_arr[a=agents, v=vertices(network)] >= 0)
    model[:vertex_time] = vertex_arrival_time

    edge_arrival_time = @variable(model, e_arr[a=agents, ed=edge_tuples] >= 0)
    model[:edge_time] = edge_arrival_time

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
        big_M=time_horizon,
    )

    # Apply objective with timing
    add_objective!(
        model,
        network,
        target_vertices,
        vertex_cost,
        edge_cost,
        vertex_select_vars,
        edge_select_vars,
        vertex_arrival_time,
        edge_arrival_time;
        with_timing=true,
    )

    # Dynamically apply conflict constraints until it's conflict-free
    while is_binary

        # Set start values from previous optimization
        set_start_value.(all_vars, current_solution)

        # Run optimization
        optimize!(model)
        @assert termination_status(model) == OPTIMAL

        all_vars = all_variables(model)
        current_solution = value.(all_vars)

        valid_vertices, valid_edges = parse_result(model)
        vertex_signal = detect_vertex_conflict(valid_vertices, valid_edges)
        if vertex_signal != DEFAULT_VERTEX_CONFLICT_SIGNAL
            add_vertex_conflict!(
                model,
                network,
                vertex_signal.v,
                vertex_signal.a1,
                vertex_signal.a2,
                vertex_arrival_time,
                edge_arrival_time,
                edge_select_vars,
                time_horizon;
                binary=is_binary,
                heuristic_conflict=heuristic_conflict,
                safety_gap=epsilon,
            )
            continue
        end

        edge_signal = detect_edge_conflict(
            valid_vertices, valid_edges; detect_swap=swap_constraint
        )
        if edge_signal != DEFAULT_EDGE_CONFLICT_SIGNAL
            add_edge_conflict!(
                model,
                edge_signal.e,
                edge_signal.a1,
                edge_signal.a2,
                edge_signal.swap,
                vertex_arrival_time,
                edge_arrival_time;
                time_horizon=time_horizon,
                binary=is_binary,
                heuristic_conflict=heuristic_conflict,
                safety_gap=epsilon,
            )
            continue
        end

        break
    end

    if termination_status(model) != OPTIMAL
        optimize!(model)
    end

    return (parse_result(model)..., objective_value(model))
end
