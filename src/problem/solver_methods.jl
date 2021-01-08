################################################################################
# Solver Methods
################################################################################

function newton_solve!(prob::GameProblem{KN,n,m,T,SVd,SVx}) where {KN,n,m,T,SVd,SVx}# vis::Visualizer=Visualizer(), live_vis::Bool=false)
	model = prob.model
	core = prob.core
	game_con = prob.game_con
	opts = prob.opts

	# Set initial trajectory
	Random.seed!(100)
	init_traj!(prob.pdtraj; x0=prob.x0, f=opts.f_init, amplitude=opts.amplitude_init)
	init_traj!(prob.pdtraj_trial; x0=prob.x0, f=opts.f_init, amplitude=opts.amplitude_init)
	init_traj!(prob.Δpdtraj; x0=prob.x0, f=zeros, amplitude=0.0)

	# Set the initial penalties
	prob.pen.ρ = SVector{1,T}([opts.ρ_0])
	prob.pen.ρ_trial = SVector{1,T}([opts.ρ_trial])

	# Reset Statistics and constraints
	reset!(prob.stats)
	reset!(game_con)
	# Iterative solve
    for k = 1:opts.outer_iter
		#XXX vio = evaluate_constraints(model, pdtraj, verbose=true)
		#XXX dist = ResidualDistribution13(res)
		#XXX record!(prob.stats, vio, dist)
		record!(prob.stats)

		# Initialize regularization and failed line search count.
		set!(opts.reg, opts.reg_0)
		LS_count = 0
		opts.inner_print ? display_solver_header() : nothing
		#XXX XXX opts.inner_print ? display_condition_header() : nothing
        for l = 1:opts.inner_iter
			set!(opts.reg, opts.reg_0*l^4)
			LS_count, control_flow = inner_iteration(prob, LS_count, k, l)
			LS_count >= 3 || control_flow == :break ? break : nothing
		end

		# Visualize trajectory
		#XXX live_vis && (opts.outer_iter-k)%1==0 ? visualize!(vis, model, pdtraj.q) : nothing
		# Kick out before updating the penalty and duals
        k == opts.outer_iter ? break : nothing
		# Dual Ascent
		#XXX dual_ascent!(game_con, pdtraj)
		# Increasing Schedule
		prob.pen.ρ = min.(prob.pen.ρ * opts.ρ_increase, opts.ρ_max)
		penalty_update!(game_con)
    end
	#XXX vio = evaluate_constraints(model, pdtraj, verbose=true)
	#XXX dist = ResidualDistribution13(res)
	#XXX record!(prob.stats, vio, dist)
	record!(prob.stats)
    return nothing
end

function inner_iteration(prob::GameProblem, LS_count::Int, k::Int, l::Int)
	core = prob.core
	opts = prob.opts

	# Residual
	residual!(prob, prob.pdtraj)
	#XXX XXX opts.regularize ? regularize_residual!(res, opts, pdtraj, pdtraj) : nothing # should do nothing since we regularize around pdtraj
	# loss = [L_opt(cost,con_traj,model,opts,pdtraj),
	# 		L_mdp(model,opts,pdtraj),
	# 		L_nf(model,opts,pdtraj)]
	res_norm = norm(core.res, 1)/length(core.res)
	if res_norm <= 1e-6
		return LS_count, :break
	end
	# Residual Jacobian
	residual_jacobian!(prob)
	regularize_residual_jacobian!(prob)

	Δtraj = - \(lu(core.jac), core.res)
	set_traj!(core, prob.Δpdtraj, Δtraj)

	# Line Search
	α, j = line_search(prob, res_norm)
	update_traj!(prob.pdtraj, prob.pdtraj, α, prob.Δpdtraj)

	failed_ls = j == opts.ls_iter
	failed_ls ? LS_count += 1 : LS_count = 0
	# condi = cond(Array(core.jac))
	opts.inner_print ? display_solver_data(k, l, j, res_norm, opts.reg) : nothing
	#XXX XXX opts.inner_print ? display_condition_data(res) : nothing
	return LS_count, :continue
end

function line_search(prob::GameProblem, res_norm::T) where {T}
	core = prob.core
	opts = prob.opts

	j = 1
	α = 1.0
	while j < opts.ls_iter
		update_traj!(prob.pdtraj_trial, prob.pdtraj, α, prob.Δpdtraj)
		residual!(prob, prob.pdtraj_trial)
		#XXX XXX opts.regularize ? regularize_residual!(res, opts, pdtraj_trial, pdtraj) : nothing# should add regularization term to prevent deviation from pdtraj
		res_norm_trial = norm(core.res, 1)/length(core.res)
		if res_norm_trial <= (1.0-α*opts.β)*res_norm
			LS_count = 0
			break
		else
			α *= opts.α_decrease # step size decrease
			j += 1
		end
	end
	return α, j
end
