@testset "Stamp" begin
    # Test Stamo and VStamp
    using Test
    N = 10
    p = 3
    vstamp1 = VStamp(:opt1, :x, 5)
    @test valid(vstamp1,N,p)

    vstamp1 = VStamp(:opt4, :x, 5)
    @test !valid(vstamp1,N,p)

    vstamp1 = VStamp(:opt2, :u, 10)
    @test !valid(vstamp1,N,p)

    vstamp1 = VStamp(:opt2, :z, 1)
    @test !valid(vstamp1,N,p)

    vstamp1 = VStamp(:dyn, :u, 1)
    @test !valid(vstamp1,N,p)

    vstamp1 = VStamp(:dyn, :x, 1)
    @test valid(vstamp1,N,p)

    vstamp1 = VStamp(:opt2, :u, 4)
    @test valid(vstamp1,N,p)

    stamp1 = Stamp(:opt1, :x, 5, :x, 3)
    @test valid(stamp1,N,p)

    stamp1 = Stamp(:opt1, :x, 5, :u, 3)
    @test !valid(stamp1,N,p)

    stamp1 = Stamp(:opt1, :x, 5, :u3, 3)
    @test valid(stamp1,N,p)

    stamp1 = Stamp(:opt1, :x, 5, :λ3, 3)
    @test !valid(stamp1,N,p)

    stamp1 = Stamp(:opt1, :x, 5, :λ1, 3)
    @test valid(stamp1,N,p)

    stamp1 = Stamp(:dyn, :x, 5, :x, 3)
    @test valid(stamp1,N,p)

    stamp1 = Stamp(:dyn, :x, 5, :u2, 3)
    @test valid(stamp1,N,p)

    # Test equality
    stamp1 = Stamp(:opt1, :x, 5, :λ1, 3)
    stamp2 = Stamp(:opt1, :x, 5, :λ1, 3)
    stamp3 = Stamp(:opt2, :x, 5, :λ1, 3)
    @test stamp1 == stamp2
    @test !(stamp1 == stamp3)
    @test !(Stamp() == VStamp())
    vstamp1 = VStamp()
    vstamp2 = VStamp()
    vstamp3 = VStamp(:opt2, :x, 3)
    @test vstamp1 == vstamp2
    @test !(vstamp1 == vstamp3)

    # Test Stampify
    stamp0 = stampify(:opt1, :x_1, 6, :λ1, 3)
    stamp1 = stampify(:opt1, :x, 5, :λ1, 3)
    stamp2 = stampify(:opt1, :x1, 4, :λ1, 3)
    @test stamp0 == stamp1
    @test stamp1 == stamp2

    stampify!(stamp2, :opt2, :x1, 4, :λ1, 3)
    @test !(stamp1 == stamp2)

    vstamp0 = VStamp()
    vstamp1 = stampify(:opt1, :x, 5)
    stampify!(vstamp0, :opt1, :x, 5)
    @test vstamp0 == vstamp1

end
