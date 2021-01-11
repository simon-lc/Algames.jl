################################################################################
# Trajectory Plot
################################################################################

function plot_traj!(model::AbstractGameModel, traj::Traj)
    plt = plot(aspect_ratio=:equal)
    N = length(traj)
    for i = 1:model.p
        xi = [state(traj[k])[model.pz[i][1]] for k=1:N]
        yi = [state(traj[k])[model.pz[i][2]] for k=1:N]
        plot!(xi, yi)
        scatter!(xi, yi)
    end
    display(plt)
    return nothing
end

################################################################################
# Constraint Violation Plot
################################################################################

function plot_violation!(stats::Statistics; lw::T=5.0) where {T}
	plt = plot(
		size=(500,500),
		layout=(1,1,))
    iter = stats.iter
    dyn = log.(10, max.(1e-10, [stats.dyn_vio[i].max for i=1:iter]))
    con = log.(10, max.(1e-10, [stats.con_vio[i].max for i=1:iter]))
    sta = log.(10, max.(1e-10, [stats.sta_vio[i].max for i=1:iter]))
    opt = log.(10, max.(1e-10, [stats.opt_vio[i].max for i=1:iter]))
	plot!(plt[1,1],
		legend=:bottomleft,
		xlabel="Outer Loop Iterations",
		ylabel="log(cons. vio.)",
		title="Constraint Violation")

	plot!(plt[1,1], dyn, linewidth=lw, label="dyn", legend=:bottomleft)
	plot!(plt[1,1], con, linewidth=lw, label="con", legend=:bottomleft)
	plot!(plt[1,1], sta, linewidth=lw, label="sta", legend=:bottomleft)
	plot!(plt[1,1], opt, linewidth=lw, label="opt", legend=:bottomleft)
    display(plt)
    return nothing
end
