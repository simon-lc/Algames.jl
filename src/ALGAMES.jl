module ALGAMES

greet() = print("Hello World!")

using BenchmarkTools
using ForwardDiff
using LinearAlgebra
using Parameters
using Plots
using Printf
using Random
using RobotDynamics
using SparseArrays
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

# Struct
export
    ProblemSize,
    PrimalDualTraj,
    init_traj!,
    update_traj!,
    set_traj!,
    Statistics,
    record!,
    reset!,
    dynamics_violation

# Core
export
    NewtonCore,
    vertical_indices,
    horizontal_indices,
    idx,
    horizontal_idx,
    vertical_idx,
    residual_views,
    jacobian_views,
    dynamics_indices,
    VStamp,
    Stamp,
    stampify,
    stampify!,
    valid

# Problem
export
    Regularizer,
    set!,
    mult!,
    Options,
    GameObjective,
    cost_gradient!,
    cost_hessian!,
    GameConstraintList,
    GameProblem,
    residual!,
    residual_jacobian!,
    add2sub,
    addI2sub,
    sparse_zero!,
    dynamics_residual,
    ∇dynamics!,
    newton_solve!,
    inner_iteration,
    line_search

# Plots
export
    plot_traj

include("newcode.jl")

# Dynamics
include("dynamics/game_model.jl")
include("dynamics/double_integrator.jl")
include("dynamics/unicycle.jl")

# Struct
include("struct/problem_size.jl")
include("struct/statistics.jl")

# Core
include("core/stamp.jl")
include("core/newton_core.jl")

# Struct
include("struct/primal_dual_traj.jl")
include("struct/violations.jl")

# Problem
include("problem/problem.jl")
include("problem/local_quantities.jl")
include("problem/global_quantities.jl")
include("problem/solver_methods.jl")

# Plots
include("plots/solver_plots.jl")

end # module
