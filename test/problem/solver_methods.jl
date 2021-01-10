@testset "Solver Methods" begin

    # Test Sover Methods
    # Test solver on a linear unconstrained problem with one player
    # Should be solved in one iteration
    T = Float64
    N = 20
    dt = 0.1
    p = 1
    model = DoubleIntegratorGame(p=p)
    probsize = ProblemSize(N,model)
    x0 = SVector{model.n,T}([1.0, 1.0, 0.0, 0.9])
    opts = Options()
    opts.inner_print = false
    opts.outer_print = false

    Q = [Diagonal(1*ones(SVector{model.ni[i],T})) for i=1:p]
    R = [Diagonal(0.5*ones(SVector{model.mi[i],T})) for i=1:p]
    xf = [0*ones(SVector{model.ni[i],T}) for i=1:p]
    uf = [-1*ones(SVector{model.mi[i],T}) for i=1:p]
    game_obj = GameObjective(Q,R,xf,uf,N,model)

    game_con = GameConstraintValues(probsize)
    prob = GameProblem(N, dt, x0, model, opts, game_obj, game_con)

    opts.outer_iter = 1
    opts.inner_iter = 1
    opts.ls_iter = 25
    opts.reg_0 = 1e-7
    newton_solve!(prob)
    @test norm(prob.core.res, 1)/length(prob.core.res) < 1e-6
    @test dynamics_violation(model, prob.pdtraj).max < 1e-6

    # Test solver on a non linear unconstrained problem with one player
    T = Float64
    N = 20
    dt = 0.1
    p = 1
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N,model)
    x0 = SVector{model.n,T}([1.0, 1.0, 0.0, 0.9])
    opts = Options()
    opts.inner_print = false
    opts.outer_print = false

    Q = [Diagonal(1*ones(SVector{model.ni[i],T})) for i=1:p]
    R = [Diagonal(0.5*ones(SVector{model.mi[i],T})) for i=1:p]
    xf = [0*ones(SVector{model.ni[i],T}) for i=1:p]
    uf = [-1*ones(SVector{model.mi[i],T}) for i=1:p]
    game_obj = GameObjective(Q,R,xf,uf,N,model)

    game_con = GameConstraintValues(probsize)
    prob = GameProblem(N, dt, x0, model, opts, game_obj, game_con)

    opts.outer_iter = 7
    opts.inner_iter = 20
    opts.ls_iter = 25
    opts.reg_0 = 1e-7
    newton_solve!(prob)
    @test norm(prob.core.res, 1)/length(prob.core.res) < 1e-6
    @test dynamics_violation(model, prob.pdtraj).max < 1e-6


    # Test solver on a linear unconstrained problem with 2 players
    T = Float64
    N = 20
    dt = 0.1
    p = 2
    model = DoubleIntegratorGame(p=p)
    probsize = ProblemSize(N,model)
    x0 = SVector{model.n,T}([1.0, 2.0, 1.0, 2.0, 0.0, 0.0, 0.9, 0.9])
    opts = Options()
    opts.inner_print = false
    opts.outer_print = false

    Q = [Diagonal(1*ones(SVector{model.ni[i],T})) for i=1:p]
    R = [Diagonal(0.5*ones(SVector{model.mi[i],T})) for i=1:p]
    xf = [0*ones(SVector{model.ni[i],T}) for i=1:p]
    uf = [-1*ones(SVector{model.mi[i],T}) for i=1:p]
    game_obj = GameObjective(Q,R,xf,uf,N,model)

    game_con = GameConstraintValues(probsize)
    prob = GameProblem(N, dt, x0, model, opts, game_obj, game_con)

    opts.outer_iter = 1
    opts.inner_iter = 1
    opts.ls_iter = 25
    opts.reg_0 = 1e-7
    newton_solve!(prob)
    @test norm(prob.core.res, 1)/length(prob.core.res) < 1e-6
    @test dynamics_violation(model, prob.pdtraj).max < 1e-6


    # Test solver on a non linear unconstrained problem with 2 players
    T = Float64
    N = 20
    dt = 0.1
    p = 2
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N,model)
    x0 = SVector{model.n,T}([1.0, 2.0, 1.0, 2.0, 0.0, 0.0, 0.9, 0.9])
    opts = Options()
    opts.inner_print = false
    opts.outer_print = false

    Q = [Diagonal(1*ones(SVector{model.ni[i],T})) for i=1:p]
    R = [Diagonal(0.5*ones(SVector{model.mi[i],T})) for i=1:p]
    xf = [0*ones(SVector{model.ni[i],T}) for i=1:p]
    uf = [-1*ones(SVector{model.mi[i],T}) for i=1:p]
    game_obj = GameObjective(Q,R,xf,uf,N,model)

    game_con = GameConstraintValues(probsize)
    prob = GameProblem(N, dt, x0, model, opts, game_obj, game_con)

    opts.outer_iter = 7
    opts.inner_iter = 20
    opts.ls_iter = 25
    opts.reg_0 = 1e-7
    newton_solve!(prob)
    @test norm(prob.core.res, 1)/length(prob.core.res) < 1e-6
    @test dynamics_violation(model, prob.pdtraj).max < 1e-6


end
