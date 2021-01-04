using Test
using ALGAMES
using BenchmarkTools
using ForwardDiff
using LinearAlgebra
using Parameters
using Printf
using SparseArrays
using StaticArrays
using Random
using RobotDynamics
using TrajectoryOptimization

include("vec_addsub.jl")
include("VecPair.jl")

# Dynamics
include("dynamics/double_integrator.jl")
include("dynamics/unicycle.jl")

# Struct
include("struct/problem_size.jl")
include("struct/primal_dual_traj.jl")
include("struct/statistics.jl")

# Core
include("core/stamp.jl")
include("core/newton_core.jl")

# Problem
include("problem/problem.jl")
include("problem/local_quantities.jl")
include("problem/global_quantities.jl")
include("problem/solver_methods.jl")
