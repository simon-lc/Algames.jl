################################################################################
# Penalty
################################################################################

mutable struct Penalty{T}
	ρ::SVector{1,T}
	ρ_trial::SVector{1,T}
end

function Penalty(ρ::T) where {T}
	return Penalty(SVector{1,T}(ρ), SVector{1,T}(ρ))
end

################################################################################
# GameObjective
################################################################################

mutable struct GameObjective
	p::Int                 # number of players
	obj::Vector{Objective} # objective of each player
	E::Vector{Objective}   # quadratic expansion of the objective of each player
end

function GameObjective(Q::Vector{DQ}, R::Vector{DR}, xf::Vector{SVx}, uf::Vector{SVu},
	N::Int, model::AbstractGameModel) where {DQ,DR,SVx,SVu}
	T = eltype(xf[1])
	n = model.n
	m = model.m
	p = model.p
	@assert length(Q) == length(R) == length(xf) == length(uf)

	obj = Vector{Objective}(undef, p)
	E = Vector{Objective}(undef, p)
	for i = 1:p
		Qi = Diagonal(SVector{n,T}(expand_vector(diag(Q[i]), model.pz[i], n)))
		Ri = Diagonal(SVector{m,T}(expand_vector(diag(R[i]), model.pu[i], m)))
		Rf = Diagonal(zeros(SVector{m,T}))
		xfi = SVector{n,T}(expand_vector(xf[i], model.pz[i], n))
		ufi = SVector{m,T}(expand_vector(uf[i], model.pu[i], m))
		cost = LQRCost(Qi,Ri,xfi,ufi,checks=false)
		cost_term = LQRCost(Qi,Rf,xfi,checks=false)
		obj[i] = Objective(cost, cost_term, N)
		E[i] = Objective([LQRCost(1e-10*ones(n,n)+I, 1e-10*zeros(m,m)+I, zeros(MVector{n,T})) for k=1:N])
	end
	return GameObjective(p,obj,E)
end

function expand_vector(v::AbstractVector{T}, inds::AbstractVector{Int}, n::Int) where {T}
	V = zeros(n)
	V[inds] = v
	return V
end

function cost_gradient!(game_obj::GameObjective, pdtraj::PrimalDualTraj)
	p = game_obj.p
	for i = 1:p
		TrajectoryOptimization.cost_gradient!(game_obj.E[i], game_obj.obj[i], pdtraj.pr, true)
	end
	return nothing
end

function cost_hessian!(game_obj::GameObjective, pdtraj::PrimalDualTraj)
	p = game_obj.p
	for i = 1:p
		TrajectoryOptimization.cost_hessian!(game_obj.E[i], game_obj.obj[i], pdtraj.pr, true, true)
	end
	return nothing
end

################################################################################
# GameProblem
################################################################################

mutable struct GameProblem{KN,n,m,T,SVd,SVx}
    probsize::ProblemSize
	model::AbstractGameModel
    core::NewtonCore
	x0::SVx
    game_obj::GameObjective
    game_con::GameConstraintValues
	pdtraj::PrimalDualTraj{KN,n,m,T,SVd}
	pdtraj_trial::PrimalDualTraj{KN,n,m,T,SVd}
    Δpdtraj::PrimalDualTraj{KN,n,m,T,SVd}
	pen::Penalty{T}
    opts::Options
	stats::Statistics
end

function GameProblem(N::Int, dt::T, x0::SVx, model::AbstractGameModel, opts::Options,
	game_obj::GameObjective, game_con::GameConstraintValues
	) where {T,SVx}

	probsize = ProblemSize(N,model)
	pdtraj = PrimalDualTraj(probsize, dt)
	pdtraj_trial = PrimalDualTraj(probsize, dt)
	Δpdtraj = PrimalDualTraj(probsize, dt)
	core = NewtonCore(probsize)
	stats = Statistics()
	pen = Penalty(opts.ρ_0)
	# Set the constraint parameters according to the options selected for the problem.
	set_constraint_params!(game_con, opts)

	TYPE = (eltype(pdtraj.pr), model.n, model.m, T, eltype(pdtraj.du), typeof(x0))
	return GameProblem{TYPE...}(probsize, model, core, x0, game_obj, game_con,
		pdtraj, pdtraj_trial, Δpdtraj, pen, opts, stats)
end
