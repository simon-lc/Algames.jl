################################################################################
# PrimalDualTraj
################################################################################

mutable struct PrimalDualTraj{KN,n,m,T,SVd}
    probsize::ProblemSize
    pr::Traj{n,m,T,KN} # Primal trajectory
    du::Vector{SVd} # Dual trajectory
end

function PrimalDualTraj(probsize::ProblemSize, dt::T; f=rand, amplitude=1e-8) where {T}
    N = probsize.N
    n = probsize.n
    m = probsize.m
    p = probsize.p
    pr = Traj(n,m,dt,N)
    du = [[amplitude*f(SVector{n,T}) for k=1:N-1] for i=1:p]
    for k = 1:N
        pr[k].z = amplitude*f(n+m)
    end
    TYPE = (eltype(pr),n,m,T,eltype(du))
    return PrimalDualTraj{TYPE...}(probsize, pr, du)
end

################################################################################
# Methods
################################################################################

function init_traj!(pdtraj::PrimalDualTraj{KN,n,m,T,SVd}; x0=rand(SVector{n,T}),
    f=rand, amplitude=1e-8) where {KN,n,m,T,SVd}
    N = pdtraj.probsize.N
    p = pdtraj.probsize.p

    for k = 1:N
        pdtraj.pr[k].z = amplitude*f(SVector{n+m,T})
    end
    for i = 1:p
        for k = 1:N-1
            pdtraj.du[i][k] = amplitude*f(SVector{n,T})
        end
    end
    RobotDynamics.set_state!(pdtraj.pr[1], x0)
    return nothing
end

function set_traj!(core::NewtonCore, Δpdtraj::PrimalDualTraj{KN,n,m,T,SVd},
    Δtraj::AbstractVector) where {KN,n,m,T,SVd}
    N = core.probsize.N
    p = core.probsize.p
    # Primals
    for k = 1:N-1
        # States
        ind = horizontal_idx(core, :x, 1, k+1)
        RobotDynamics.set_state!(Δpdtraj.pr[k+1], Δtraj[ind])
        # Controls
        ind = vcat([horizontal_idx(core, :u, i, k) for i=1:p]...)
        RobotDynamics.set_control!(Δpdtraj.pr[k], Δtraj[ind])
    end
    # Duals
    for i = 1:p
        for k = 1:N-1
            ind = horizontal_idx(core, :λ, i, k)
            Δpdtraj.du[i][k] = Δtraj[ind]
        end
    end
    return nothing
end

function update_traj!(target::PrimalDualTraj{KN,n,m,T,SVd},
    source::PrimalDualTraj{KN,n,m,T,SVd}, α::T,
    Δ::PrimalDualTraj{KN,n,m,T,SVd}) where {KN,n,m,T,SVd,SVx}
    N = target.probsize.N
    p = target.probsize.p
    # Primals
    for k = 1:N-1
        # States
        RobotDynamics.set_state!(target.pr[k+1], state(source.pr[k+1]) + α*state(Δ.pr[k+1]))
        # Controls
        RobotDynamics.set_control!(target.pr[k], control(source.pr[k+1]) + α*control(Δ.pr[k]))
    end
    # Duals
    for i = 1:p
        for k = 1:N-1
            target.du[i][k] = source.du[i][k] + α*Δ.du[i][k]
        end
    end
    return nothing
end
