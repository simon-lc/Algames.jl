################################################################################
# Residual
################################################################################

function residual!(prob::GameProblem{KN,n,m,T,SVd,SVx}) where {KN,n,m,T,SVd,SVx}
	N = prob.probsize.N
    p = prob.probsize.p
	core = prob.core
	model = prob.model
	pdtraj = prob.pdtraj
    stamp = VStamp()

    for k = 1:N-1
        # Local Lagrangians
        for i = 1:p
            # xx
        end
        # Dynamics
        stampify!(stamp, :dyn, 1, :x, 1, k)
        add2sub(core.res_sub[stamp], dynamics_residual(model, pdtraj, k)) # shouldn't allocate
    end
    return nothing
end

################################################################################
# Residual Jacobian
################################################################################

function residual_jacobian!(prob::GameProblem{KN,n,m,T,SVd,SVx}) where {KN,n,m,T,SVd,SVx}
    N = prob.probsize.N
    p = prob.probsize.p
	model = prob.model
    core = prob.core

    # Allocations
    stamp = Stamp()
    ∇dyn = zeros(MMatrix{n,(n+m),T,n*(n+m)})

    for k = 1:N-1
        # Lagrangians
        for i = 1:p
            # xx
        end
        # Dynamics
        ∇dynamics!(∇dyn, model, prob.pdtraj, k)
        # Bottom Left
        stampify!(stamp, :dyn, 1, :x, 1, k, :x, 1, k)
		∇dyn_x = ∇dyn[:,core.dyn[:x][1]]
		valid(stamp, N, p) ? add2sub(core.jac_sub[stamp], ∇dyn[:,core.dyn[:x][1]]) : nothing
        for i = 1:p
            stampify!(stamp, :dyn, 1, :x, 1, k, :u, i, k)
            valid(stamp, N, p) ? add2sub(core.jac_sub[stamp], ∇dyn[:,core.dyn[:u][i]]) : nothing
        end
        stampify!(stamp, :dyn, 1, :x, 1, k, :x, 1, k+1)
        valid(stamp, N, p) ? addI2sub(core.jac_sub[stamp], -1.0) : nothing
        # Top Right
        for i = 1:p
            stampify!(stamp, :opt, i, :x, 1, k, :λ, i, k)
            valid(stamp, N, p) ? add2sub(core.jac_sub[stamp], ∇dyn[:,core.dyn[:x][1]]') : nothing
            stampify!(stamp, :opt, i, :u, i, k, :λ, i, k)
            valid(stamp, N, p) ? add2sub(core.jac_sub[stamp], ∇dyn[:,core.dyn[:u][i]]') : nothing
            stampify!(stamp, :opt, i, :x, 1, k+1, :λ, i, k)
            valid(stamp, N, p) ? addI2sub(core.jac_sub[stamp], -1.0) : nothing
        end
    end
    return nothing
end
################################################################################
# Helpers
################################################################################

function add2sub(v::SubArray, e)
	v .+= e
	return nothing
end

function add2sub(v::SubArray, e::Diagonal{T,SVector{n,T}}) where {n,T}
	for i = 1:n
		v[i,i] += e[i,i]
	end
	return nothing
end

function addI2sub(v::SubArray, e)
	n = size(v)[1]
	for i = 1:n
		v[i,i] += e
	end
	return nothing
end




#
#
# T = Float64
# model = UnicycleGame(p=3)
# x = rand(SVector{model.n,T})
# u = rand(SVector{model.m,T})
# dt = 0.2
# k = KnotPoint(x,u,dt)
# @ballocated discrete_dynamics(RK2, $model, $k)
# @btime discrete_dynamics($RK4, $model, $k)
#
# v = view(∇f, (1:12), (1:18))
# @ballocated RobotDynamics.jacobian!($∇f,$model,$k)
# @btime RobotDynamics.jacobian!($∇f,$model,$k)
# @ballocated RobotDynamics.jacobian!($v,$model,$k)
# @btime RobotDynamics.jacobian!($v,$model,$k)
