@testset "Game Constraints" begin

    # Test GameConstraintValues
    N = 10
    model = UnicycleGame(p=3)
    probsize = ProblemSize(N,model)
    game_con = GameConstraintValues(probsize)
    @test game_con.p == model.p
    @test length(game_con.state_conlist) == model.p

    # Test set_constraint_params!
    T = Float64
    u_max =  rand(SVector{model.m,T})
    u_min = -rand(SVector{model.m,T})
    add_control_bound!(game_con, probsize, u_max, u_min)
    radius = 1.0
    add_collision_avoidance!(game_con, probsize, radius)
    opts = Options{T}()
    opts.ρ_increase = 2.0
    opts.ρ_0 = 3.0
    opts.ρ_max = 4.0
    opts.λ_max = 5.0
    set_constraint_params!(game_con, opts)
    @test game_con.state_conval[model.p][1].params.ϕ == 2.0
    @test game_con.state_conval[model.p][1].params.μ0 == 3.0
    @test game_con.state_conval[model.p][1].params.μ_max == 4.0
    @test game_con.state_conval[model.p][1].params.λ_max == 5.0

    @test game_con.control_conval[1].params.ϕ == 2.0
    @test game_con.control_conval[1].params.μ0 == 3.0
    @test game_con.control_conval[1].params.μ_max == 4.0
    @test game_con.control_conval[1].params.λ_max == 5.0


end
