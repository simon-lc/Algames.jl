function residual!(prob::GameProblem)
    # N = prob.probsize.N
    # stamp = VStamp()
    # for k = 1:N-1
    #     # Local Lagrangians
    #     for i = 1:p
    #         xx
    #     end
    #     # Dynamics
    #     stampify!(stamp, :dyn, :x, k)
    #     add2sub(prob.res_sub[stamp], dynamics())
    # end
    return nothing
end


function jacobian!(prob::GameProblem)


    return nothing
end




#
# model = UnicycleGame(p=3)
# x = rand(SVector{model.n,T})
# u = rand(SVector{model.m,T})
# dt = 0.2
# k = KnotPoint(x,u,dt)
# @ballocated discrete_dynamics(RK2, $model, $k)
# @btime discrete_dynamics($RK4, $model, $k)
