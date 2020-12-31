using Test
using ALGAMES
using BenchmarkTools
using LinearAlgebra
using SparseArrays
using StaticArrays
using TrajectoryOptimization

include("vec_addsub.jl")
include("VecPair.jl")

# Dynamics
include("dynamics/double_integrator.jl")
include("dynamics/unicycle.jl")

# Struct
include("struct/problem_size.jl")
include("struct/statistics.jl")

# Core
include("core/stamp.jl")
include("core/newton_core.jl")

# Problem
include("problem/problem.jl")
include("problem/local_quantities.jl")
include("problem/global_quantities.jl")
include("problem/solver.jl")
