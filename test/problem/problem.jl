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

    # Test GameConstraintList
    N = 10
    n = 12
    m = 6
    p = 3
    conlists = [ConstraintList(n,m,N) for i=1:p]
    game_conlist = GameConstraintList(conlists)
    @test game_conlist.p == p
    @test game_conlist.conlist == conlists

end
