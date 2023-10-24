using Documenter
using MultiAgentPathFindingMILP

DocMeta.setdocmeta!(
    MultiAgentPathFindingMILP,
    :DocTestSetup,
    :(using MultiAgentPathFindingMILP);
    recursive=true,
)

makedocs(;
    sitename="MultiAgentPathFindingMILP.jl",
    authors="Chun-Tso Tsai",
    format=Documenter.HTML(; prettyurls=get(ENV, "CI", "false") == "true"),
    modules=[MultiAgentPathFindingMILP],
    pages=["Home" => "index.md", "Theories" => "theories.md", "Algorithms" => "algorithms.md"],
)

# Documenter can also automatically deploy documentation to gh-pages.
# See "Hosting Documentation" and deploydocs() in the Documenter manual
# for more information.
#=deploydocs(
    repo = "<repository url>"
)=#
