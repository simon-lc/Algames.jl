@testset "Global Quantities" begin

    # Test Residual
    T = Float64
    N = 10
    dt = 0.1
    p = 3
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N,model)
    x0 = rand(SVector{model.n,T})
    opts = Options()

    Q = [Diagonal(rand(SVector{model.ni[i],T})) for i=1:p]
    R = [Diagonal(rand(SVector{model.mi[i],T})) for i=1:p]
    xf = [i*ones(SVector{model.ni[i],T}) for i=1:p]
    uf = [2i*ones(SVector{model.mi[i],T}) for i=1:p]
    game_obj = GameObjective(Q,R,xf,uf,N,model)

    game_con = GameConstraintValues(probsize)
    prob = GameProblem(N, dt, x0, model, opts, game_obj, game_con)
    @test typeof(prob) <: GameProblem
    residual!(prob)

    # Test Residual Jacobian
    residual_jacobian!(prob)

end
