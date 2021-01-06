@testset "Problem" begin

    # Test Options
    opts = Options()
    @test typeof(opts) <: Options

    # Test GameObjective
    v = [1,2,3]
    inds = [1,3,5]
    n = 5
    V = [1,0,2,0,3]
    @test V == ALGAMES.expand_vector(v,inds,n)

    # Test Game Objective
    T = Float64
    N = 10
    n = 12
    m = 6
    p = 3
    model = UnicycleGame(p=p)
    Q = [Diagonal(rand(SVector{model.ni[i],T})) for i=1:p]
    R = [Diagonal(rand(SVector{model.mi[i],T})) for i=1:p]
    xf = [i*ones(SVector{model.ni[i],T}) for i=1:p]
    uf = [2i*ones(SVector{model.mi[i],T}) for i=1:p]

    game_obj = GameObjective(Q,R,xf,uf,N,model)

    X = [1.0*SVector{model.n,T}([1,2,3,1,2,3,1,2,3,1,2,3]) for k=1:N]
    U = [1.0*SVector{model.m,T}([2,4,6,2,4,6]) for k=1:N-1]
    dt = 0.1
    Dt = dt*ones(N-1)
    traj = Traj(X, U,Dt)

    @test abs(cost(game_obj.obj[1], traj)) <= 1e-10
    @test abs(cost(game_obj.obj[2], traj)) <= 1e-10
    @test abs(cost(game_obj.obj[3], traj)) <= 1e-10

    # Test cost_gradient! and cost_hessian!
    probsize = ProblemSize(N, model)
    pdtraj = PrimalDualTraj(probsize, dt, f=rand, amplitude=1e1)
    for i = 1:p
        for k = 1:N
            game_obj.E[i].cost[k].Q += rand(n,n)
            game_obj.E[i].cost[k].R += rand(m,m)
            game_obj.E[i].cost[k].H += rand(m,n)
            game_obj.E[i].cost[k].q += rand(n)
            game_obj.E[i].cost[k].r += rand(m)
            game_obj.E[i].cost[k].c += rand()
        end
    end
    cost_gradient!(game_obj, pdtraj)
    cost_hessian!(game_obj, pdtraj)

    # Gradient
    # Stage cost
    @test norm(game_obj.E[1].cost[1].q - (game_obj.obj[1].cost[1].Q*state(pdtraj.pr[1]) + game_obj.obj[1].cost[1].q)*dt, 1) < 1e-10
    @test norm(game_obj.E[1].cost[1].r - (game_obj.obj[1].cost[1].R*control(pdtraj.pr[1]) + game_obj.obj[1].cost[1].r)*dt, 1) < 1e-10
    # Terminal cost
    @test norm(game_obj.E[1].cost[end].q - (game_obj.obj[1].cost[end].Q*state(pdtraj.pr[end]) + game_obj.obj[1].cost[end].q), 1) < 1e-10
    @test norm(game_obj.E[1].cost[end].r, 1) < 1e-10

    # Hessian
    # Stage cost
    @test norm(game_obj.E[1].cost[1].Q - game_obj.obj[1].cost[1].Q*dt, 1) < 1e-10
    @test norm(game_obj.E[1].cost[1].R - game_obj.obj[1].cost[1].R*dt, 1) < 1e-10
    # Terminal cost
    @test norm(game_obj.E[1].cost[end].Q - game_obj.obj[1].cost[end].Q, 1) < 1e-10
    @test norm(game_obj.E[1].cost[end].R, 1) < 1e-10


    # Test GameConstraintValues
    N = 10
    model = UnicycleGame(p=3)
    probsize = ProblemSize(N,model)
    game_con = GameConstraintValues(probsize)
    @test game_con.p == model.p
    @test length(game_con.state_conlist) == model.p

    # Test GameProblem
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

end
