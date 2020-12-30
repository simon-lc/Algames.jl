module ALGAMES

greet() = print("Hello World!")

using BenchmarkTools
using LinearAlgebra
using Parameters
using StaticArrays
using RobotDynamics
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

# Problem
export
    ProblemSize,
    Options,
    GameObjective

include("newcode.jl")
# Dynamics
include("dynamics/game_model.jl")
include("dynamics/double_integrator.jl")
include("dynamics/unicycle.jl")

# Problem
include("problem/problem.jl")

end # module
