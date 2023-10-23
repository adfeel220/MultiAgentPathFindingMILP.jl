
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

end
