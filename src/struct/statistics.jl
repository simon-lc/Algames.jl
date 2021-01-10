################################################################################
# Statistics
################################################################################

mutable struct Statistics
	iter::Int
	dyn_vio::AbstractVector{DynamicsViolation}
	con_vio::AbstractVector{ControlViolation}
	sta_vio::AbstractVector{StateViolation}
end

function Statistics()
	iter = 0
	dyn_vio = Vector{DynamicsViolation}()
	con_vio = Vector{ControlViolation}()
	sta_vio = Vector{StateViolation}()
	return Statistics(iter, dyn_vio, con_vio, sta_vio)
end

function record!(stats::Statistics, dyn_vio::DynamicsViolation,
	con_vio::ControlViolation, sta_vio::StateViolation)
	stats.iter += 1
	push!(stats.dyn_vio, dyn_vio)
	push!(stats.con_vio, con_vio)
	push!(stats.sta_vio, sta_vio)
	return nothing
end

function record!(stats::Statistics, model::AbstractGameModel,
	game_con::GameConstraintValues, pdtraj::PrimalDualTraj)

	stats.iter += 1
	push!(stats.dyn_vio, dynamics_violation(model, pdtraj))
	push!(stats.con_vio, control_violation(game_con, pdtraj))
	push!(stats.sta_vio, state_violation(game_con, pdtraj))
	return nothing
end

function reset!(stats::Statistics)
	stats_ = Statistics()
	for name in fieldnames(Statistics)
		setfield!(stats, name, getfield(stats_, name))
	end
	return nothing
end
