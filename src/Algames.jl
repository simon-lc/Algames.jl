module Algames

greet() = print("Hello World!")

using Altro
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

# Constraints
export
    ControlBoundConstraint,
    WallConstraint,
    add_collision_avoidance!,
    add_control_bound!,
    add_circle_constraint!,
    Wall,
    add_wall_constraint!,
    set_constraint_params!,
    reset!, # need test
    reset_duals!, # need test
    reset_penalties!, # need test
    penalty_update!, # need test
    constraint_jacobian_residual!,
    constraint_residual!

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
    Penalty,
    Regularizer,
    set!,
    mult!,
    Options,
    GameObjective,
    cost_gradient!,
    cost_hessian!,
    GameConstraintValues,
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
include("struct/regularizer.jl")
include("struct/options.jl")

# Constraints
include("constraints/control_bound_constraint.jl")
include("constraints/wall_constraint.jl")
include("constraints/game_constraints.jl")
include("constraints/constraints_methods.jl")

# Problem
include("problem/problem.jl")
include("problem/local_quantities.jl")
include("problem/global_quantities.jl")
include("problem/solver_methods.jl")

# Constraints
include("constraints/constraint_derivatives.jl")

# Plots
include("plots/solver_plots.jl")

end # module
