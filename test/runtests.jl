using MultiAgentPathFindingMILP
using Test
using Aqua
using JET
using Documenter
using JuliaFormatter

@testset verbose = true "MultiAgentPathFindingMILP.jl" begin

    # Code format
    @testset "Code formatting" begin
        @test format(MultiAgentPathFindingMILP; verbose=false, overwrite=false)
    end

    if VERSION >= v"1.9"
        @testset "Code quality" begin
            Aqua.test_all(MultiAgentPathFindingMILP; ambiguities=false)
        end

        @testset "Code linting" begin
            JET.test_package(MultiAgentPathFindingMILP; target_defined_modules=true)
        end
    end

    @testset "Doctests" begin
        doctest(MultiAgentPathFindingMILP)
    end

    @testset verbose = true "Algorithms" begin
        @testset "Conflict" begin
            include("conflict.jl")
        end

        @testset "Continuous Time" begin
            include("continuous_time.jl")
        end
    end
end
