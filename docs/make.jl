import Pkg
Pkg.activate(@__DIR__)
Pkg.develop(Pkg.PackageSpec(path = joinpath(@__DIR__, "..")))
Pkg.instantiate()

using Documenter
using Literate
using PowerConverterDynamics

DocMeta.setdocmeta!(PowerConverterDynamics, :DocTestSetup, :(using PowerConverterDynamics); recursive = true)

generated_src_dir = joinpath(@__DIR__, "src", "generated")
mkpath(generated_src_dir)

Literate.markdown(
    joinpath(@__DIR__, "literate", "converter_battery_dynamics.jl"),
    generated_src_dir;
    name = "converter_battery_dynamics",
    documenter = true,
)

makedocs(
    sitename = "PowerConverterDynamics.jl",
    modules = [PowerConverterDynamics],
    remotes = nothing,
    format = Documenter.HTML(
        prettyurls = get(ENV, "CI", "false") == "true",
        edit_link = "master",
        repolink = "https://github.com/sandialabs/PowerConverterDynamics.jl",
    ),
    checkdocs = :exports,
    pages = [
        "Home" => "index.md",
        "Quickstart" => "quickstart.md",
        "Usage" => "usage.md",
        "Examples" => [
            "Overview" => "examples.md",
            "Literate Walkthrough" => "generated/converter_battery_dynamics.md",
        ],
        "Theory" => "theory.md",
        "API" => "api.md",
    ],
)

if get(ENV, "CI", "false") == "true"
    deploydocs(
        repo = "github.com/sandialabs/PowerConverterDynamics.jl.git",
        devbranch = "master",
    )
end
