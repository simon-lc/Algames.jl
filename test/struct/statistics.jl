@testset "Statistics" begin

    # Test Statistics
    stats = Statistics()
    N = 10
    dt = 0.1
    model = UnicycleGame(p=3)
    probsize = ProblemSize(N,model)

    dyn_vio = DynamicsViolation(N)
    con_vio = ControlViolation(N)
    sta_vio = StateViolation(N)
    record!(stats, dyn_vio, con_vio, sta_vio)
    @test stats.iter == 1

    game_con = GameConstraintValues(probsize)
    u_max =  0.1*ones(model.m)
    u_min = -0.1*ones(model.m)
    add_control_bound!(game_con, probsize, u_max, u_min)
    walls = [Wall([0.,1], [1,0], [1,1]/sqrt(2))]
    add_wall_constraint!(game_con, probsize, walls)
    pdtraj = PrimalDualTraj(probsize, dt)
    record!(stats, model, game_con, pdtraj)
    @test stats.iter == 2

    reset!(stats)
    @test stats.iter == 0

end
