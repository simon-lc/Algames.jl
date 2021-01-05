@testset "Violations" begin

    # Test dynamics violation
    T = Float64
    N = 10
    dt = 0.1
    model = UnicycleGame(p=3)
    probsize = ProblemSize(N, model)
    pdtraj = PrimalDualTraj(probsize, dt)

    init_traj!(pdtraj, x0=zeros(SVector{model.n,T}), f=zeros)
    @test dynamics_violation(model, pdtraj) == 0.0
    init_traj!(pdtraj, x0=ones(SVector{model.n,T}), f=ones, amplitude=1.0)
    @test dynamics_violation(model, pdtraj) - maximum(abs.(dynamics_residual(model, pdtraj, 1))) < 1e-10

end
