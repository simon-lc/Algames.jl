################################################################################
# Trajectory Plot
################################################################################

function plot_traj!(model::AbstractGameModel, traj::Traj)
    plt = plot(aspect_ratio=:equal)
    N = length(traj)
    for i = 1:model.p
        @show model.pz[i][1]
        @show model.pz[i][2]
        xi = [state(traj[k])[model.pz[i][1]] for k=1:N]
        yi = [state(traj[k])[model.pz[i][2]] for k=1:N]
        plot!(xi, yi)
        scatter!(xi, yi)
    end
    display(plt)
    return nothing
end
