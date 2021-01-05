@testset "Constraint Methods" begin

    # Test Collision Avoidance
    T = Float64
    N = 20
    dt = 0.1
    p = 3
    model = DoubleIntegratorGame(p=p)
    probsize = ProblemSize(N,model)
    conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
    game_conlist = GameConstraintList(conlists)
    radius = 1.0
    add_collision_avoidance!(game_conlist, probsize, radius)

    pu = model.pu
    @test game_conlist.conlist[1].constraints[1].x1 == pu[1]
    @test game_conlist.conlist[1].constraints[2].x1 == pu[1]
    @test game_conlist.conlist[1].constraints[1].x2 == pu[2]
    @test game_conlist.conlist[1].constraints[2].x2 == pu[3]

    @test game_conlist.conlist[2].constraints[1].x1 == pu[2]
    @test game_conlist.conlist[2].constraints[2].x1 == pu[2]
    @test game_conlist.conlist[2].constraints[1].x2 == pu[1]
    @test game_conlist.conlist[2].constraints[2].x2 == pu[3]

    @test game_conlist.conlist[3].constraints[1].x1 == pu[3]
    @test game_conlist.conlist[3].constraints[2].x1 == pu[3]
    @test game_conlist.conlist[3].constraints[1].x2 == pu[1]
    @test game_conlist.conlist[3].constraints[2].x2 == pu[2]


    # Test Control Bounds
    T = Float64
    N = 20
    dt = 0.1
    p = 3
    model = DoubleIntegratorGame(p=p)
    probsize = ProblemSize(N,model)
    conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
    game_conlist = GameConstraintList(conlists)
    u_min = -10*ones(model.m)
    u_max =  10*ones(model.m)
    u_min[1] = -Inf
    u_max[1] =  Inf
    add_control_bound!(game_conlist, probsize, u_max, u_min)

    @test game_conlist.conlist[1].constraints[1].u_min == u_min
    @test game_conlist.conlist[1].constraints[1].u_max == u_max

    @test game_conlist.conlist[2].constraints[1].u_min == u_min
    @test game_conlist.conlist[2].constraints[1].u_max == u_max

    @test game_conlist.conlist[3].constraints[1].u_min == u_min
    @test game_conlist.conlist[3].constraints[1].u_max == u_max

    # Test Circle Constraint
    T = Float64
    N = 20
    dt = 0.1
    p = 3
    model = DoubleIntegratorGame(p=p)
    px = model.px
    probsize = ProblemSize(N,model)
    conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
    game_conlist = GameConstraintList(conlists)
    P = 5
    xc = SVector{P,T}([1.0, 2.0, 3.0, 4.0, 5.0])
    yc = SVector{P,T}([-1.0, -2.0, -3.0, -4.0, -5.0])
    radius = SVector{P,T}([0.1, 0.2, 0.3, 0.4, 0.5])
    add_circle_constraint!(game_conlist, probsize, xc, yc, radius)

    @test game_conlist.conlist[1].constraints[1].xi == px[1][1]
    @test game_conlist.conlist[1].constraints[1].yi == px[1][2]

    @test game_conlist.conlist[2].constraints[1].xi == px[2][1]
    @test game_conlist.conlist[2].constraints[1].yi == px[2][2]

    @test game_conlist.conlist[3].constraints[1].xi == px[3][1]
    @test game_conlist.conlist[3].constraints[1].yi == px[3][2]


    # Test WallConstraint
    T = Float64
    N = 20
    dt = 0.1
    p = 3
    model = DoubleIntegratorGame(p=p)
    px = model.px
    n = model.n
    probsize = ProblemSize(N,model)
    conlists = [ConstraintList(model.n,model.m,N) for i=1:p]
    game_conlist = GameConstraintList(conlists)
    P = 5
    x = 4
    y = 2
    x1 = SVector{P,T}([ 0.0,  0.0,  1.0,  3.0, -2.0])
    y1 = SVector{P,T}([ 1.0, -1.0,  2.0,  2.0,  0.0])
    x2 = SVector{P,T}([ 1.0,  1.0,  2.0,  2.0,  0.0])
    y2 = SVector{P,T}([ 0.0,  0.0,  1.0,  1.0,  0.0])
    xv = SVector{P,T}([ 1.0,  1.0,  1.0,  1.0,  0.0])./sqrt(2)
    yv = SVector{P,T}([ 1.0, -1.0,  1.0, -1.0,  sqrt(2)])./sqrt(2)

    con = WallConstraint(n,x1,y1,x2,y2,xv,yv,x,y)
    walls = [Wall([x1[j],y1[j]], [x2[j],y2[j]], [xv[j],yv[j]]) for j=1:P]
    add_wall_constraint!(game_conlist, probsize, walls )

    @test game_conlist.conlist[1].constraints[1].x == px[1][1]
    @test game_conlist.conlist[1].constraints[1].y == px[1][2]

    @test game_conlist.conlist[2].constraints[1].x == px[2][1]
    @test game_conlist.conlist[2].constraints[1].y == px[2][2]

    @test game_conlist.conlist[3].constraints[1].x == px[3][1]
    @test game_conlist.conlist[3].constraints[1].y == px[3][2]

end
