using Test
using ALGAMES
using BenchmarkTools
using LinearAlgebra
using StaticArrays
using TrajectoryOptimization

include("vec_addsub.jl")
include("VecPair.jl")

# Dynamics
include("dynamics/double_integrator.jl")
include("dynamics/unicycle.jl")

# Core
include("core/newton_core.jl")
include("core/stamp.jl")

# Problem
include("problem/problem.jl")
