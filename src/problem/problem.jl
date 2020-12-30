################################################################################
# ProblemSize
################################################################################

mutable struct ProblemSize{SVu,SVx,SVz}
    N::Int
    n::Int
    m::Int
    p::Int
    ni::Vector{Int}
    mi::Vector{Int}
    pu::SVu
    px::SVx
    pz::SVz
end

function ProblemSize(N::Int, model::AbstractGameModel)
    return ProblemSize(
        N,
        model.n,
        model.m,
        model.p,
        model.ni,
        model.mi,
        model.pu,
        model.px,
        model.pz,
        )
end

import Base.==
function (==)(p1::ProblemSize, p2::ProblemSize)
    out = true
    for name in fieldnames(ProblemSize)
        out &= getfield(p1, name) == getfield(p2, name)
    end
    return out
end

################################################################################
# GameOptions
################################################################################

@with_kw mutable struct Options{T}
    # Options
	"Gauss-newton convergence criterion: tolerance."
	θ::T=1e-2

	# Regularization
	"Regularization of the residual and residual Jacobian."
	regularize::Bool=true

	# "Current Jacobian regularization for each primal-dual variables."
	# reg::Regularizer{T}=Regularizer()

	"Initial Jacobian regularization."
	reg_0::T=1e-3

	# Line search
	"Initial step size."
	α_0::T=1.0

	"Line search increase step."
	α_increase::T=1.2

	"Line search decrease step."
	α_decrease::T=0.5

	"Expected residual improvement."
	β::T=0.01

	"Number of line search iterations."
	ls_iter::Int=25

	# Augmented Lagrangian Penalty
	"Initial augmented Lagrangian penalty."
	ρ_0::T=1.0

	"Fixed augmented Lagrangian penalty on the residual used for the line search trials."
	ρ_trial::T=1.0

	"Penalty increase step."
	ρ_increase::T=10.0

	"Maximum augmented Lagrangian penalty."
	ρ_max::T=1e7

	# Augmented Lagrangian iterations.
	"Outer loop iterations."
	outer_iter::Int=7

	"Inner loop iterations."
	inner_iter::Int=20

	# Problem Scaling
	"Objective function scaling."
	γ::T=1e0

	# MPC
	"Number of time steps simulated with MPC"
	mpc_horizon::Int=20

	"Rate of the upsampling used for simulaion and feedback control in the MPC"
	upsampling::Int=2

	# Printing
	"Displaying the inner step results."
	inner_print::Bool=true

	"Displaying the outer step results."
	outer_print::Bool=true
end

################################################################################
# GameObjective
################################################################################

mutable struct GameObjective
	p::Int
	obj::Vector{Objective}
end

function GameObjective(Q::Vector{DQ}, R::Vector{DR}, xf::Vector{SVx}, uf::Vector{SVu},
	N::Int, model::AbstractGameModel) where {DQ,DR,SVx,SVu}
	T = eltype(xf[1])
	n = model.n
	m = model.m
	p = model.p
	@assert length(Q) == length(R) == length(xf) == length(uf)

	obj = Vector{Objective}(undef, p)
	for i = 1:p
		Qi = Diagonal(SVector{n,T}(expand_vector(diag(Q[i]), model.pz[i], n)))
		Ri = Diagonal(SVector{m,T}(expand_vector(diag(R[i]), model.pu[i], m)))
		Rf = Diagonal(zeros(SVector{m,T}))
		xfi = SVector{n,T}(expand_vector(xf[i], model.pz[i], n))
		ufi = SVector{m,T}(expand_vector(uf[i], model.pu[i], m))
		cost = LQRCost(Qi,Ri,xfi,ufi,checks=false)
		cost_term = LQRCost(Qi,Rf,xfi,checks=false)
		obj[i] = Objective(cost, cost_term, N)
	end
	return GameObjective(p,obj)
end

function expand_vector(v::AbstractVector{T}, inds::AbstractVector{Int}, n::Int) where {T}
	V = zeros(n)
	V[inds] = v
	return V
end


################################################################################
# GameProblem
################################################################################

mutable struct GameProblem12{SVx}
    probsize::ProblemSize
    # core::NewtonCore
	x0::SVx
    game_obj::GameObjective
    # con::Vector{ConstraintSet}
    traj::Traj
    opts::Options
end

function GameProblem12(N::Int, dt::T, x0::SVx, model::AbstractGameModel, opts::Options,
	# cost::GameCost, con::GameConstraintSet
	) where {T,SVx}

	probsize = ProblemSize(N,model)
	traj = Trajectory(model.n,model.m,dt,N)

	TYPE = typeof.((x0))
	return GameProblem12{TYPE...}(probsize, core, x0, game_obj, con, traj, opts)
end



# T = Float64
# N = 10
# n = 12
# m = 6
# p = 3
# model = UnicycleGame(p=p)
# Q = [Diagonal(rand(SVector{model.ni[i],T})) for i=1:p]
# R = [Diagonal(rand(SVector{model.mi[i],T})) for i=1:p]
# xf = [i*ones(SVector{model.ni[i],T}) for i=1:p]
# uf = [2i*ones(SVector{model.mi[i],T}) for i=1:p]
#
# game_obj = GameObjective(Q,R,xf,uf,N,model)
#
# X = [1.0*SVector{model.n,T}([1,2,3,1,2,3,1,2,3,1,2,3]) for k=1:N]
# U = [1.0*SVector{model.m,T}([2,4,6,2,4,6]) for k=1:N-1]
# dt = 0.1
# Dt = dt*ones(N-1)
# traj = Traj(X, U,Dt)
#
# @test cost(game_obj.obj[1], traj) == 0.0
# @test cost(game_obj.obj[2], traj) == 0.0
# @test cost(game_obj.obj[3], traj) == 0.0
