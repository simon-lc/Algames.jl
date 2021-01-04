@testset "Primal Dual Traj" begin

    # Test PrimalDualTraj
    N = 10
    dt = 0.2
    p = 3
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N, model)
    pdtraj = PrimalDualTraj(probsize, dt)
    @test size(pdtraj.pr)[1] == N
    @test size(state(pdtraj.pr.data[1]))[1] == model.n
    @test size(control(pdtraj.pr.data[1]))[1] == model.m
    @test pdtraj.pr.data[1].dt == dt
    @test size(pdtraj.du)[1] == p
    @test size(pdtraj.du[1])[1] == N-1
    @test size(pdtraj.du[1][1])[1] == model.n

    # Test init_traj!
    T = Float64
    x0 = rand(SVector{model.n,T})
    init_traj!(pdtraj, x0=x0, f=ones, amplitude=10.0)
    @test state(pdtraj.pr[1]) == x0
    @test state(pdtraj.pr[2]) == 10*ones(model.n)
    @test control(pdtraj.pr[1]) == 10*ones(model.m)
    @test pdtraj.du[1][1] == 10*ones(model.n)
    @test pdtraj.du[1][1] == 10*ones(model.n)

    # Test set_traj!
    n = model.n
    m = model.m
    p = model.p
    x0 = rand(SVector{model.n,T})
    Δpdtraj = PrimalDualTraj(probsize, dt)
    Δtraj = ones(n*(N-1)+m*(N-1)+n*p*(N-1))
    core = NewtonCore(probsize)
    init_traj!(Δpdtraj, x0=x0, f=ones, amplitude=10.0)
    set_traj!(core, Δpdtraj, Δtraj)

    @test state(Δpdtraj.pr[1]) == x0
    @test state(Δpdtraj.pr[2]) == ones(n)
    @test state(Δpdtraj.pr[end]) == ones(n)
    @test control(Δpdtraj.pr[1]) == ones(m)
    @test control(Δpdtraj.pr[end-1]) == ones(m)
    @test Δpdtraj.du[1][1] == ones(n)
    @test Δpdtraj.du[1][end] == ones(n)
    @test Δpdtraj.du[end][end] == ones(n)

    # Test update_traj!
    n = model.n
    m = model.m
    p = model.p
    x0 = rand(SVector{model.n,T})
    target = PrimalDualTraj(probsize, dt)
    source = PrimalDualTraj(probsize, dt)
    Δ = PrimalDualTraj(probsize, dt)
    init_traj!(target, x0=x0, f=ones, amplitude=0.0)
    init_traj!(source, x0=x0, f=ones, amplitude=10.0)
    init_traj!(Δ, x0=x0, f=ones, amplitude=100.0)
    α = 0.5
    update_traj!(target, source, α, Δ)

    @test state(target.pr[1]) == x0
    @test state(target.pr[2]) == 60*ones(n)
    @test state(target.pr[end]) == 60*ones(n)
    @test control(target.pr[1]) == 60*ones(m)
    @test control(target.pr[end-1]) == 60*ones(m)
    @test target.du[1][1] == 60*ones(n)
    @test target.du[1][end] == 60*ones(n)
    @test target.du[end][end] == 60*ones(n)

end
# N = 10
# dt = 0.2
# p = 3
# model = UnicycleGame(p=p)
# probsize = ProblemSize(N, model)
# pdtraj = PrimalDualTraj(probsize, dt)
# @test size(pdtraj.pr)[1] == N
# @test size(state(pdtraj.pr.data[1]))[1] == model.n
# @test size(control(pdtraj.pr.data[1]))[1] == model.m
# @test pdtraj.pr.data[1].dt == dt
# @test size(pdtraj.du)[1] == p
# @test size(pdtraj.du[1])[1] == N-1
# @test size(pdtraj.du[1][1])[1] == model.n
#
# # Test init_traj!
# T = Float64
# x0 = rand(SVector{model.n,T})
# init_traj!(pdtraj, x0=x0, f=ones, amplitude=10.0)
# @test state(pdtraj.pr[1]) == x0
# @test state(pdtraj.pr[2]) == 10*ones(model.n)
# @test control(pdtraj.pr[1]) == 10*ones(model.m)
# @test pdtraj.du[1][1] == 10*ones(model.n)
# @test pdtraj.du[1][1] == 10*ones(model.n)
#
# # Test set_traj!
# n = model.n
# m = model.m
# p = model.p
# x0 = rand(SVector{model.n,T})
# Δpdtraj = PrimalDualTraj(probsize, dt)
# Δtraj = ones(n*(N-1)+m*(N-1)+n*p*(N-1))
# core = NewtonCore(probsize)
# init_traj!(Δpdtraj, x0=x0, f=ones, amplitude=10.0)
# set_traj!(core, Δpdtraj, Δtraj)
#
# @test state(Δpdtraj.pr[1]) == x0
# @test state(Δpdtraj.pr[2]) == ones(n)
# @test state(Δpdtraj.pr[end]) == ones(n)
# @test control(Δpdtraj.pr[1]) == ones(m)
# @test control(Δpdtraj.pr[end-1]) == ones(m)
# @test Δpdtraj.du[1][1] == ones(n)
# @test Δpdtraj.du[1][end] == ones(n)
# @test Δpdtraj.du[end][end] == ones(n)
#
# # Test update_traj!
# n = model.n
# m = model.m
# p = model.p
# x0 = rand(SVector{model.n,T})
# target = PrimalDualTraj(probsize, dt)
# source = PrimalDualTraj(probsize, dt)
# Δ = PrimalDualTraj(probsize, dt)
# init_traj!(target, x0=x0, f=ones, amplitude=0.0)
# init_traj!(source, x0=x0, f=ones, amplitude=10.0)
# init_traj!(Δ, x0=x0, f=ones, amplitude=100.0)
# α = 0.5
# update_traj!(target, source, α, Δ)
#
# @test state(target.pr[1]) == x0
# @test state(target.pr[2]) == 60*ones(n)
# @test state(target.pr[end]) == 60*ones(n)
# @test control(target.pr[1]) == 60*ones(m)
# @test control(target.pr[end-1]) == 60*ones(m)
# @test target.du[1][1] == 60*ones(n)
# @test target.du[1][end] == 60*ones(n)
# @test target.du[end][end] == 60*ones(n)
