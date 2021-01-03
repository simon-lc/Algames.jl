################################################################################
# Dynamics Residual
################################################################################

function dynamics_residual(model::AbstractGameModel, pdtraj::PrimalDualTraj{KN}, k::Int) where {KN}
    zk1 = pdtraj.pr[k+1]::KN
    xk1 = state(zk1)
    zk = pdtraj.pr[k]::KN
    return dynamics_residual(model, zk, xk1)
end

function dynamics_residual(model::AbstractGameModel, zk::AbstractKnotPoint, xk1::SVx) where {SVx}
    return discrete_dynamics(RK2, model, zk) - xk1
end

################################################################################
# Dynamics Jacobian
################################################################################

function ∇dynamics!(∇f::SM, model::AbstractGameModel, pdtraj::PrimalDualTraj{KN}, k::Int) where {KN,SM}
    zk = pdtraj.pr[k]::KN
    return ∇dynamics!(∇f, model, zk)
end

function ∇dynamics!(∇f::SM, model::AbstractGameModel, zk::AbstractKnotPoint) where {SM}
    return RobotDynamics.discrete_jacobian!(RK2, ∇f, model, zk)
end

################################################################################
# Objective Evaluations
################################################################################

# T = Float64
# N = 10
# dt = 0.1
# n = 12
# m = 6
# p = 3
# model = UnicycleGame(p=p)
# probsize = ProblemSize(N,model)
# Q = [Diagonal(i*ones(SVector{model.ni[i],T})) for i=1:p]
# R = [Diagonal(10*i*ones(SVector{model.mi[i],T})) for i=1:p]
# xf = [i*ones(SVector{model.ni[i],T}) for i=1:p]
# uf = [2i*ones(SVector{model.mi[i],T}) for i=1:p]
#
# game_obj = GameObjective(Q,R,xf,uf,N,model)
# obj1 = game_obj.obj[1]
# obj2 = game_obj.obj[2]
#
# E1 = Objective(LQRCost(ones(n,n), ones(m,m), ones(n)), N)
# E1 = Objective([LQRCost(ones(MMatrix{n,n,T,n^2}), ones(SMatrix{m,m,T,m^2}), ones(SVector{n,T})) for k = 1:N])
# E2 = Objective(LQRCost(ones(MMatrix{n,n,T,n^2}), ones(MMatrix{m,m,T,m^2}), ones(MVector{n,T})), N)
#
#
# pdtraj.pr
# pdtraj = PrimalDualTraj(probsize, dt, f=rand, amplitude=1e2)
# TrajectoryOptimization.cost_gradient!(E1, obj1, pdtraj.pr)
# TO.cost_hessian!(E1, obj1, pdtraj.pr, true, true)
# E1.cost[3]
# TrajectoryOptimization.hessian!(E1[1], obj1[1], x, u)
# E1.cost[1]
# TrajectoryOptimization.is_diag(obj1[1])
#
# control(pdtraj.pr[4])
#
# x = rand(SVector{n,T})
# u = rand(SVector{m,T})
# TrajectoryOptimization.hessian!(E1.cost[1], obj1.cost[1], x, u)
# TrajectoryOptimization.hessian!(E2.cost[1], obj2.cost[1], x, u)
# E1.cost[1]
# E2.cost[1]
#
#
# TrajectoryOptimization.TO.QuadraticCostFunction(a,)
# ################################################################################
# # Objective Gradients
# ################################################################################
#
# ################################################################################
# # Objective Hessians
# ################################################################################
