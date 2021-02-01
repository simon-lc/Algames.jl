################################################################################
# Trajectory Plot
################################################################################

function plot_traj!(model::AbstractGameModel, traj::Traj; plt=Plots.plot())
    Plots.plot!(plt, aspect_ratio=:equal)
    N = length(traj)
	c = [:orange, :cornflowerblue, :forestgreen, :red, :black, :pink]
    for i = 1:model.p
        xi = [state(traj[k])[model.pz[i][1]] for k=1:N]
        yi = [state(traj[k])[model.pz[i][2]] for k=1:N]
        Plots.plot!(xi, yi, color=c[i%6])
        Plots.scatter!(xi, yi, color=c[i%6])
    end
    Plots.display(plt)
    return nothing
end

################################################################################
# Constraint Violation Plot
################################################################################

function plot_violation!(stats::Statistics; plt=Plots.plot(), lw::T=5.0) where {T}
	Plots.plot!(plt,
		size=(500,500),
		layout=(1,1,))
    iter = stats.iter
    dyn = log.(10, max.(1e-10, [stats.dyn_vio[i].max for i=1:iter]))
    con = log.(10, max.(1e-10, [stats.con_vio[i].max for i=1:iter]))
    sta = log.(10, max.(1e-10, [stats.sta_vio[i].max for i=1:iter]))
    opt = log.(10, max.(1e-10, [stats.opt_vio[i].max for i=1:iter]))
	y_min = minimum([dyn; con; sta; opt])
	y_max = maximum([dyn; con; sta; opt])
	# Set up plot
	Plots.plot!(plt[1,1],
		legend=:bottomleft,
		xlabel="Outer Loop Iterations",
		ylabel="log(cons. vio.)",
		title="Constraint Violation")
	# Add curves
	Plots.plot!(plt[1,1], dyn, linewidth=lw, label="dyn", legend=:bottomleft, color=:green)
	Plots.plot!(plt[1,1], con, linewidth=lw, label="con", legend=:bottomleft, color=:blue)
	Plots.plot!(plt[1,1], sta, linewidth=lw, label="sta", legend=:bottomleft, color=:orange)
	Plots.plot!(plt[1,1], opt, linewidth=lw, label="opt", legend=:bottomleft, color=:red)
	# Add rectangles
	plot_epochs!(plt, y_min, y_max, stats.outer_iter)

    Plots.display(plt)
    return nothing
end

function plot_epochs!(plt, y_min::T, y_max::T, epochs::Vector{Int}) where {T}
	rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])
	i_start = 1
	i_end = -1
	for k = 1:epochs[end]
		i_end = findlast(x -> x==k, epochs )
		Plots.plot!(rectangle(i_end-i_start,y_max-y_min,i_start,y_min), opacity=.1, label=false)
		i_start = i_end + 1
	end
	return nothing
end
