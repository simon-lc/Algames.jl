@testset "Global Quantities" begin

    # Test Residual
    T = Float64
    N = 10
    dt = 0.1
    p = 3
    model = UnicycleGame(p=p)
    x0 = rand(SVector{model.n,T})
    opts = Options()

    Q = [Diagonal(rand(SVector{model.ni[i],T})) for i=1:p]
    R = [Diagonal(rand(SVector{model.mi[i],T})) for i=1:p]
    xf = [i*ones(SVector{model.ni[i],T}) for i=1:p]
    uf = [2i*ones(SVector{model.mi[i],T}) for i=1:p]
    game_obj = GameObjective(Q,R,xf,uf,N,model)

    conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
    game_conlist = GameConstraintList(conlists)
    prob = GameProblem(N, dt, x0, model, opts, game_obj, game_conlist)
    @test typeof(prob) <: GameProblem
    residual!(prob)

    # Test Residual Jacobian
    residual_jacobian!(prob)

end

# Test Residual
T = Float64
N = 10
dt = 0.1
p = 3
model = UnicycleGame(p=p)
x0 = rand(SVector{model.n,T})
opts = Options()

Q = [Diagonal(rand(SVector{model.ni[i],T})) for i=1:p]
R = [Diagonal(rand(SVector{model.mi[i],T})) for i=1:p]
xf = [i*ones(SVector{model.ni[i],T}) for i=1:p]
uf = [2i*ones(SVector{model.mi[i],T}) for i=1:p]
game_obj = GameObjective(Q,R,xf,uf,N,model)

conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
game_conlist = GameConstraintList(conlists)
prob = GameProblem(N, dt, x0, model, opts, game_obj, game_conlist)
@test typeof(prob) <: GameProblem
residual!(prob)

# Test Residual Jacobian
residual_jacobian!(prob)





@btime residual!(prob)
@ballocated residual!(prob)

@btime residual_jacobian!(prob)
@ballocated residual_jacobian!(prob)






# # Test Residual
# T = Float64
# N = 10
# dt = 0.1
# p = 3
# model = UnicycleGame(p=p)
# x0 = rand(SVector{model.n,T})
# opts = Options()
#
# Q = [Diagonal(rand(SVector{model.ni[i],T})) for i=1:p]
# R = [Diagonal(rand(SVector{model.mi[i],T})) for i=1:p]
# xf = [i*ones(SVector{model.ni[i],T}) for i=1:p]
# uf = [2i*ones(SVector{model.mi[i],T}) for i=1:p]
# game_obj = GameObjective(Q,R,xf,uf,N,model)
#
# conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
# game_conlist = GameConstraintList(conlists)
# prob = GameProblem(N, dt, x0, model, opts, game_obj, game_conlist)
# @test typeof(prob) <: GameProblem
# residual!(prob)
#
#
# residual_jacobian!(prob)
# # @btime residual!(prob)
# @ballocated residual_jacobian!(prob)
# @btime residual_jacobian!(prob)
#
# stamp = stampify(:dyn, :x, 1, :x, 2)
# @ballocated valid(stamp, N,p)
#
# function tt()
#     i = 1::Int
#     a = "x$i"
#     return a
# end
#
# @ballocated tt()
#
#
#
# T = Float64
# N = 10
# dt = 0.1
# p = 3
# model = UnicycleGame(p=p)
# x0 = rand(SVector{model.n,T})
# opts = Options()
#
# Q = [Diagonal(rand(SVector{model.ni[i],T})) for i=1:p]
# R = [Diagonal(rand(SVector{model.mi[i],T})) for i=1:p]
# xf = [i*ones(SVector{model.ni[i],T}) for i=1:p]
# uf = [2i*ones(SVector{model.mi[i],T}) for i=1:p]
# game_obj = GameObjective(Q,R,xf,uf,N,model)
#
# conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
# game_conlist = GameConstraintList(conlists)
# prob = GameProblem(N, dt, x0, model, opts, game_obj, game_conlist)
# typeof(prob) <: GameProblem
#
# residual!(prob)
# @ballocated residual!($prob)
# @ballocated residual!($prob)
# @ballocated residual!($prob)
# @ballocated residual3!($prob)
# @ballocated residual5!($prob)
# @btime residual5!($prob)
#
# @code_warntype residual5!(prob)
# prob.core.res
#
#
#
# prob.core.jac_sub
#
# @ballocated dynamics_residual($model, $zk, $xk1)
# pdtraj = prob.pdtraj
# k = 5
# @ballocated dynamics_residual($model, $pdtraj, $k)
# # discrete_dynamics(RK2, model, zk)
# #
# # @code_warntype dynamics_residual6(model, pdtraj, k, tag3)
# #
# # mutable struct Tag3{T,n,m,SVz,SVx}
# #     e::Int
# # end
# #
# # mutable struct Tag4{KN,SVx}
# #     e::Int
# # end
# # mutable struct Tag5{KN}
# #     e::Int
# # end
