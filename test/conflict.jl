using MultiAgentPathFindingMILP: check_overlap_on_vertex
using Test

non_overlapping_set = 1:10
overlapping_set = [1, 2, 3, 4, 5, 2, 6, 2, 4]

@test check_overlap_on_vertex(non_overlapping_set) == false
@test check_overlap_on_vertex(overlapping_set) == true

@test check_overlap_on_vertex(non_overlapping_set; raise_assertion=true) == false
@test_throws AssertionError check_overlap_on_vertex(overlapping_set; raise_assertion=true)

@test check_overlap_on_vertex(
    non_overlapping_set, "this should not be printed"; raise_assertion=false
) == false
@test_throws AssertionError check_overlap_on_vertex(
    overlapping_set, "test_message"; raise_assertion=true
)
