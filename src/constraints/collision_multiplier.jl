################################################################################
# Collision Multiplier
################################################################################

mutable struct CollisionMultiplier{T}
    probsize::ProblemSize
    λ::Vector{Vector{Vector{T}}}
    γ::Vector{Vector{Vector{T}}}
end

function CollisionMultiplier(probsize::ProblemSize)
    N = probsize.N
    p = probsize.p
    λ = [[Vector{T}(undef, N-1) for j ∈ 1:p] for i = 1:p]
    γ = [[Vector{T}(undef, N-1) for j ∈ 1:p] for i = 1:p]
    return CollisionMultiplier{T}(probsize, λ, γ)
end

function get_collision_multiplier!(colmult::CollisionMultiplier, game_con::GameConstraintValues)
    p = probsize.p
    px = probsize.px

    for i = 1:p
        for conval in game_con.state_conval[i]
            if typeof(conval.con) <: CollisionConstraint
                @assert conval.con.x1 == px[i]
                j = findfirst(x -> all(x .== conval.con.x2), px)
                for k = 2:N
                    colmult.λ[i][j][k-1] = conval.λ[k-1][1]
                end
            end
        end
    end
    return nothing
end

function get_balance!(colmult::CollisionMultiplier, game_con::GameConstraintValues,
    core::NewtonCore, pdtraj::PrimalDualTraj)
    probsize = game_obj.probsize
    N = probsize.N
    n = probsize.n
    p = probsize.p
    pu = probsize.pu
    px = probsize.px

    cost_gradient!(game_obj, pdtraj)
    for i = 1:p
        for conval in game_con.state_conval[i]
            if typeof(conval.con) <: CollisionConstraint
                @assert conval.con.x1 == px[i]
                j = findfirst(x -> all(x .== conval.con.x2), px)
                TrajectoryOptimization.evaluate!(conval, pdtraj.pr)
                TrajectoryOptimization.jacobian!(conval, pdtraj.pr)
                TrajectoryOptimization.cost_expansion!(conval)
                for k = 2:N
                    ∇c = conval.jac[k-1] # need to use enumerate to coincide k+1 and k
                    @show ∇c
                    # ∇J = zeros(n)
                    n_obj = length(game_obj.E[i])
                    # for j = 1:n_obj
                        # ∇J += game_obj.E[i][j].cost[k].q
                    # end
                    stamp = stampify(:opt, i, :x, 1, k)
                    ∇J = core.res_sub[stamp]
                    @show inv(∇c*∇c')*∇c*∇J
                    colmult.γ[i][j][k-1] = (inv(∇c*∇c'+1e-5I)*∇c*∇J)[1]
                end
            end
        end
    end
    return nothing
end

function balance_dual!(colmult::CollisionMultiplier, game_con::GameConstraintValues)
    p = probsize.p
    px = probsize.px

    for i = 1:p
        for conval in game_con.state_conval[i]
            if typeof(conval.con) <: CollisionConstraint
                @assert conval.con.x1 == px[i]
                j = findfirst(x -> all(x .== conval.con.x2), px)
                for k = 2:N
                    γi = colmult.γ[i][j][k-1]
                    γj = colmult.γ[j][i][k-1]
                    @show γi
                    @show γj
                    s = (abs(γi*γj)+1e-3)^(-0.5)
                    @show s
                    conval.λ[k-1] .= colmult.λ[i][j][k-1] * s * γi
                end
            end
        end
    end
    return nothing
end

function unbalance_dual!(colmult::CollisionMultiplier, game_con::GameConstraintValues)
    p = probsize.p
    px = probsize.px

    for i = 1:p
        for conval in game_con.state_conval[i]
            if typeof(conval.con) <: CollisionConstraint
                @assert conval.con.x1 == px[i]
                j = findfirst(x -> all(x .== conval.con.x2), px)
                for k = 2:N
                    γi = colmult.γ[i][j][k-1]
                    γj = colmult.γ[j][i][k-1]
                    s = (abs(γi*γj)+1e-3)^(-0.5)
                    conval.λ[k-1] .= colmult.λ[i][j][k-1] / s / γi
                end
            end
        end
    end
    return nothing
end
