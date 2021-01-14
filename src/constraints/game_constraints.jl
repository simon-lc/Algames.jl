################################################################################
# GameConstraintValues
################################################################################

mutable struct GameConstraintValues{T}
	probsize::ProblemSize
	α_dual::T
	state_conlist::Vector{TrajectoryOptimization.AbstractConstraintSet}
	control_conlist::TrajectoryOptimization.AbstractConstraintSet
	state_conval::Vector{Vector{TrajectoryOptimization.AbstractConstraintValues}}
	control_conval::Vector{TrajectoryOptimization.AbstractConstraintValues}
end

function GameConstraintValues(probsize::ProblemSize)
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	α_dual = 1.0
	state_conlist = [ConstraintList(n,m,N) for i=1:p]
	control_conlist = ConstraintList(n,m,N)
	state_conval = [Vector{TrajectoryOptimization.AbstractConstraintValues}() for i=1:p]
	control_conval = Vector{TrajectoryOptimization.AbstractConstraintValues}()
	return GameConstraintValues{typeof(α_dual)}(probsize, α_dual, state_conlist,
		control_conlist, state_conval, control_conval)
end

function set_constraint_params!(game_con::GameConstraintValues, opts::Options)
	game_con.α_dual = opts.α_dual
	for i = 1:game_con.probsize.p
		for conval in game_con.state_conval[i]
			conval.params.ϕ = opts.ρ_increase
			conval.params.μ0 = opts.ρ_0
			conval.params.μ_max = opts.ρ_max
			conval.params.λ_max = opts.λ_max
		end
	end
	for conval in game_con.control_conval
		conval.params.ϕ = opts.ρ_increase
		conval.params.μ0 = opts.ρ_0
		conval.params.μ_max = opts.ρ_max
		conval.params.λ_max = opts.λ_max
	end
	return nothing
end
