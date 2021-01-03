@testset "Primal Dual Traj" begin

    # Test PrimalDualTraj
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

end
