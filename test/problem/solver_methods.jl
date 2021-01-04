
@testset "Solver Methods" begin

    # Test Sover Methods


end

# # Test solver on an unconstrained problem
# T = Float64
# N = 2
# dt = 0.1
# p = 1
# model = DoubleIntegratorGame(p=p)
# model = UnicycleGame(p=p)
# x0 = zeros(SVector{model.n,T})
# opts = Options()
#
# Q = [Diagonal(ones(SVector{model.ni[i],T})) for i=1:p]
# R = [Diagonal(ones(SVector{model.mi[i],T})) for i=1:p]
# xf = [ones(SVector{model.ni[i],T}) for i=1:p]
# uf = [0*ones(SVector{model.mi[i],T}) for i=1:p]
# game_obj = GameObjective(Q,R,xf,uf,N,model)
#
# conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
# game_conlist = GameConstraintList(conlists)
# prob = GameProblem(N, dt, x0, model, opts, game_obj, game_conlist)
#
# opts.outer_iter = 7
# opts.inner_iter = 20
# opts.ls_iter = 25
# opts.reg_0 = 1e-20
# newton_solve!(prob)
# @test norm(prob.core.res, 1)/length(prob.core.res) < 1e-6
#
# norm(prob.core.res, 1)/length(prob.core.res)
# maximum(abs.(prob.core.res))
# stamp = stampify(:dyn, 1, :x, 1, 1)
# prob.core.res_sub[stamp]
# stamp = stampify(:dyn, 1, :x, 1, 2)
# prob.core.res_sub[stamp]
# prob.pdtraj.du
#
