module ALGAMES

greet() = print("Hello World!")

using BenchmarkTools
using LinearAlgebra
using StaticArrays
using TrajectoryOptimization

export
    vec_add!,
    vec_sub!,
    VecPair

# Dynamics
export
    AbstractGameModel,
    DoubleIntegratorGame,
    UnicycleGame,
    dynamics

include("newcode.jl")
# Dynamics
include("dynamics/game_model.jl")
include("dynamics/double_integrator.jl")
include("dynamics/unicycle.jl")

end # module
