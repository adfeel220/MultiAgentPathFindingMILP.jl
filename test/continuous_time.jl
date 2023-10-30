using MultiAgentPathFindingMILP
using Graphs
using Test

#= Network is as follows

A - B - C
  /   \ |
D - E - F - G
      /
    H
=#

edge_list = Edge.([(1, 2), (2, 3), (2, 4), (2, 6), (3, 6), (4, 5), (5, 6), (6, 7), (6, 8)])
network = DiGraph(Graph(edge_list))

source_vertices = [1, 4, 8]
target_vertices = [7, 8, 3]

vertex_cost = ones(Float64, 8)
edge_cost = zeros(Float64, (8, 8))
edge_cost[1, 2] = 1.0
edge_cost[2, 3] = 1.0
edge_cost[2, 4] = 1.0
edge_cost[2, 6] = 2.0
edge_cost[3, 6] = 80.0
edge_cost[4, 5] = 20.0
edge_cost[5, 6] = 10.0
edge_cost[6, 7] = 1.0
edge_cost[6, 8] = 1.0
edge_cost .+= transpose(edge_cost)

valid_vertices, valid_edges = maph_continuous_time(
    network, source_vertices, target_vertices, vertex_cost, edge_cost
)

vertex_answer = [[1, 2, 6, 7], [4, 2, 6, 8], [8, 6, 2, 3]]
vertex_timing = [[0.0, 6.0, 9.0, 11.0], [0.0, 2.0, 7.0, 9.0], [0.0, 2.0, 5.0, 7.0]]
edge_answer = [
    [ed for ed in zip(agent_path[1:(end - 1)], agent_path[2:end])] for
    agent_path in vertex_answer
]
edge_timing = [[2.0, 7.0, 10.0], [1.0, 5.0, 8.0], [1.0, 3.0, 6.0]]

for (vertices_visited, answer, timing) in zip(valid_vertices, vertex_answer, vertex_timing)
    for ((test_time, test_v), ans_time, ans_v) in zip(vertices_visited, timing, answer)
        @test test_time ≈ ans_time
        @test test_v == ans_v
    end
end

for (edges_visited, answer, timing) in zip(valid_edges, edge_answer, edge_timing)
    for ((test_time, test_ed), ans_time, ans_ed) in zip(edges_visited, timing, answer)
        @test test_time ≈ ans_time
        @test test_ed == ans_ed
    end
end
