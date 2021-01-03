@testset "Newton Core" begin

    # Test Vertical Indices
    N = 3
    p = 2
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N,model)
    n = probsize.n
    mi = probsize.mi

    verti_inds = vertical_indices(probsize)
    @test verti_inds[stampify(:opt, 1, :x, 1, 2)] == SVector{n,Int}(1:n)
    @test verti_inds[stampify(:opt, 1, :u, 1, 1)] == SVector{mi[1],Int}(n .+ (1:mi[1]))
    @test verti_inds[stampify(:opt, 1, :x, 1, 3)] == SVector{n,Int}(n + mi[1] .+ (1:n))
    @test verti_inds[stampify(:opt, 1, :u, 1, 2)] == SVector{mi[1],Int}(2n + mi[1] .+ (1:mi[1]))
    @test verti_inds[stampify(:opt, 2, :x, 1, 2)] == SVector{n,Int}(2n+2mi[1] .+ (1:n))

    function test_vertical_indices(probsize::ProblemSize, verti_inds)
        N = probsize.N
        n = probsize.n
        m = probsize.m
        p = probsize.p
        mi = probsize.mi
        ni = probsize.ni
        all_inds = Vector{Int}([])
        for i = 1:p
            for k = 1:N-1
                push!(all_inds, verti_inds[stampify(:opt, i, :x, 1, k+1)]...)
                push!(all_inds, verti_inds[stampify(:opt, i, :u, i, k)]...)
            end
        end
        for k = 1:N-1
            push!(all_inds, verti_inds[stampify(:dyn, 1, :x, 1, k)]...)
        end
        sort!(all_inds)
        valid = true
        valid &= length(all_inds) == p*n*(N-1) + m*(N-1) + n*(N-1)
        valid &= all_inds == [i for i=1:length(all_inds)]
        return valid
    end
    @test test_vertical_indices(probsize, verti_inds)


    # Test Horizontal Indices
    N = 3
    p = 2
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N,model)
    n = probsize.n
    m = probsize.m
    mi = probsize.mi

    horiz_inds = horizontal_indices(probsize)
    @test horiz_inds[:x][1][2] == SVector{n,Int}(1:n)
    @test horiz_inds[:u][1][1] == SVector{mi[1],Int}(n .+ (1:mi[1]))
    @test horiz_inds[:u][2][1] == SVector{mi[2],Int}(n + mi[1] .+ (1:mi[2]))
    @test horiz_inds[:λ][1][1] == SVector{n,Int}(n + m .+ (1:n))
    @test horiz_inds[:λ][2][1] == SVector{n,Int}(2n + m .+ (1:n))
    @test horiz_inds[:x][1][3] == SVector{n,Int}(3n + m .+ (1:n))

    function test_horizontal_indices(probsize::ProblemSize, horiz_inds)
        N = probsize.N
        n = probsize.n
        m = probsize.m
        p = probsize.p
        mi = probsize.mi
        ni = probsize.ni
        all_inds = Vector{Int}([])
        for k = 1:N-1
            push!(all_inds, horiz_inds[:x][1][k+1]...)
            for i = 1:p
                push!(all_inds, horiz_inds[:u][i][k]...)
                push!(all_inds, horiz_inds[:λ][i][k]...)
            end
        end
        sort!(all_inds)
        valid = true
        valid &= length(all_inds) == n*(N-1) + m*(N-1) + p*n*(N-1)
        valid &= all_inds == [i for i=1:length(all_inds)]
        return valid
    end
    @test test_horizontal_indices(probsize, horiz_inds)

    # Test dynamics_indices
    N = 10
    p = 3
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N, model)
    dyn = dynamics_indices(probsize)
    n = probsize.n
    mi = probsize.mi
    @test dyn[:x][1] == SVector{n,Int}(1:n)
    @test dyn[:u][1] == SVector{mi[1],Int}(n .+ (1:mi[1]))
    @test dyn[:u][2] == SVector{mi[2],Int}(n + mi[1] .+ (1:mi[2]))
    @test dyn[:u][3] == SVector{mi[3],Int}(n + mi[1] + mi[2] .+ (1:mi[3]))

    # Test Newton Core
    N = 3
    p = 2
    model = UnicycleGame(p=p)
    probsize = ProblemSize(N,model)
    core = NewtonCore(probsize)
    @test typeof(core) <: NewtonCore

end
