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
