################################################################################
# Dynamics Violation
################################################################################

function dynamics_violation(model::AbstractGameModel, pdtraj::PrimalDualTraj)
    N = pdtraj.probsize.N
    dyn = 0.0
    for k = 1:N-1
        vio = maximum(abs.(dynamics_residual(model, pdtraj, k)))
        dyn = max(dyn, vio)
    end
    return dyn
end
