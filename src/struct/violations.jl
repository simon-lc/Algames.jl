################################################################################
# Dynamics Violation
################################################################################

mutable struct DynamicsViolation{SVd,T}
	N::Int
	vio::SVd
	max::T
end

function DynamicsViolation(N::Int)
	vio = zeros(N-1)
	max = 0.0
	TYPE = typeof.((vio,max))
	return DynamicsViolation{TYPE...}(N,vio,max)
end

function dynamics_violation(model::AbstractGameModel, pdtraj::PrimalDualTraj)
    N = pdtraj.probsize.N
    dyn_vio = DynamicsViolation(N)
    for k = 1:N-1
        dyn_vio.vio[k] = maximum(abs.(dynamics_residual(model, pdtraj, k)))
    end
	dyn_vio.max = maximum(dyn_vio.vio)
    return dyn_vio
end

################################################################################
# Control Violation
################################################################################

mutable struct ControlViolation{SVd,T}
	N::Int
	vio::SVd
	max::T
end

function ControlViolation(N::Int)
	vio = zeros(N-1)
	max = 0.0
	TYPE = typeof.((vio,max))
	return ControlViolation{TYPE...}(N,vio,max)
end

function control_violation(game_con::GameConstraintValues, pdtraj::PrimalDualTraj)
	N = pdtraj.probsize.N
	u_vio = ControlViolation(N)
	for conval in game_con.control_conval
		TrajectoryOptimization.evaluate!(conval, pdtraj.pr)
		TrajectoryOptimization.max_violation!(conval)
		u_vio.vio[conval.inds] = max.(u_vio.vio[conval.inds], conval.c_max)
	end
	u_vio.max = maximum(u_vio.vio)
    return u_vio
end

################################################################################
# State Violation
################################################################################

mutable struct StateViolation{SVd,T}
	N::Int
	vio::SVd
	max::T
end

function StateViolation(N::Int)
	vio = zeros(N)
	max = 0.0
	TYPE = typeof.((vio,max))
	return StateViolation{TYPE...}(N,vio,max)
end

function state_violation(game_con::GameConstraintValues, pdtraj::PrimalDualTraj)
	N = pdtraj.probsize.N
	p = game_con.p
	x_vio = StateViolation(N)
	for i = 1:p
		for conval in game_con.state_conval[i]
			TrajectoryOptimization.evaluate!(conval, pdtraj.pr)
			TrajectoryOptimization.max_violation!(conval)
			x_vio.vio[conval.inds] = max.(x_vio.vio[conval.inds], conval.c_max)
		end
	end
	x_vio.max = maximum(x_vio.vio)
    return x_vio
end
