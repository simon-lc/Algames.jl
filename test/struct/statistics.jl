@testset "Statistics" begin

    # Test Statistics
    stats = Statistics()
    record!(stats)
    @test stats.iter == 1
    reset!(stats)
    @test stats.iter == 0
end
