
"""
A structure to hold all necessary information for executing an MAPF solver (either continuous or discrete time)
`T`: type for waiting time;  `C`: type for costs; `D`: type for departure time. All `<:Real`
"""
@kwdef mutable struct MapfConfig{T<:Real,C<:Real,D<:Real}
    network::DiGraph
    source_vertices::Vector{Int}
    target_vertices::Vector{Int}
    vertex_cost::Array{C} = ones(Float64, nv(network))
    edge_cost::Array{C} = ones(Float64, (nv(network), nv(network)))
    vertex_wait_time::Array{T} = zeros(Float64, nv(network))
    edge_wait_time::Array{T} = ones(Float64, (nv(network), nv(network)))
    departure_time::Array{D} = zeros(Float64, length(source_vertices))
    is_binary::Bool = true
    swap_constraint::Bool = false
    time_horizon::Float64 =
        length(source_vertices) * ne(network) * maximum(edge_wait_time) +
        maximum(departure_time)
    time_duration::Int = length(source_vertices) * (ne(network) + nv(network))
    timeout::Float64 = -1.0
end
nagents(config::MapfConfig) = length(config.source_vertices)

"""
    mapf_continuous_time(config)
Execute a continuous-time MAPF with a configuration

# Arguments
- `config::MapfConfig`: Configuration struct containing all necessary information
"""
function mapf_continuous_time(config::MapfConfig; kwargs...)
    return mapf_continuous_time(
        config.network,
        config.source_vertices,
        config.target_vertices,
        config.vertex_cost,
        config.edge_cost,
        config.vertex_wait_time,
        config.edge_wait_time,
        config.departure_time;
        integer=config.is_binary,
        swap_constraint=config.swap_constraint,
        big_M=config.time_horizon,
        timeout=config.timeout,
        kwargs...
    )
end
"""
    mapf_continuous_time_dynamic_conflict(config)
Execute a dynamic constraint continuous-time MAPF with a configuration

# Arguments
- `config::MapfConfig`: Configuration struct containing all necessary information
"""
function mapf_continuous_time_dynamic_conflict(config::MapfConfig; kwargs...)
    return mapf_continuous_time_dynamic_conflict(
        config.network,
        config.source_vertices,
        config.target_vertices,
        config.vertex_cost,
        config.edge_cost,
        config.vertex_wait_time,
        config.edge_wait_time,
        config.departure_time;
        is_binary=config.is_binary,
        swap_constraint=config.swap_constraint,
        time_horizon=config.time_horizon,
        timeout=config.timeout,
        kwargs...
    )
end

"""
    mapf_discrete_time(config)
Execute a discrete-time MAPF with all the information in a config file

# Arguments
- `config::MapfConfig`: Configuration struct containing all necessary information
"""
function mapf_discrete_time(config::MapfConfig, kwargs...)
    return mapf_discrete_time(
        config.network,
        config.source_vertices,
        config.target_vertices,
        config.vertex_cost,
        config.edge_cost,
        [round(Int, dt) for dt in config.departure_time];
        integer=config.is_binary,
        time_duration=config.time_duration,
        timeout=config.timeout,
        kwargs...
    )
end
