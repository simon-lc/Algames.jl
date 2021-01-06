################################################################################
# Add Collision Avoidance
################################################################################

function add_collision_avoidance!(game_con::GameConstraintValues,
	probsize::ProblemSize, radius::T) where {T}
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	px = probsize.px
	for i = 1:p
		for j in setdiff(1:p,i)
			add_constraint!(game_con.state_conlist[i], CollisionConstraint(n,px[i],px[j],radius), 2:N)
			con  = game_con.state_conlist[i].constraints[end]
			inds = game_con.state_conlist[i].inds[end]
			conval = TrajectoryOptimization.ConVal(n,m,con,inds)
			push!(game_con.state_conval[i], conval)
		end
	end
	return nothing
end


################################################################################
# Add Control Bounds
################################################################################

function add_control_bound!(game_con::GameConstraintValues, probsize::ProblemSize,
	u_max::AbstractVector, u_min::AbstractVector)
	N = probsize.N
	n = probsize.n
	m = probsize.m
	add_constraint!(game_con.control_conlist, ControlBoundConstraint(m,u_max=u_max,u_min=u_min), 1:N-1)
	con  = game_con.control_conlist.constraints[end]
	inds = game_con.control_conlist.inds[end]
	conval = TrajectoryOptimization.ConVal(n,m,con,inds)
	push!(game_con.control_conval, conval)
	return nothing
end


################################################################################
# Add Circle Constraint
################################################################################

function add_circle_constraint!(game_con::GameConstraintValues, probsize::ProblemSize,
	xc::AbstractVector, yc::AbstractVector, radius::AbstractVector)
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	px = probsize.px
	for i = 1:p
		add_constraint!(
			game_con.state_conlist[i],
			TrajectoryOptimization.CircleConstraint(n, xc, yc, radius, px[i][1], px[i][2]),
			2:N)
		con  = game_con.state_conlist[i].constraints[end]
		inds = game_con.state_conlist[i].inds[end]
		conval = TrajectoryOptimization.ConVal(n,m,con,inds)
		push!(game_con.state_conval[i], conval)
	end
	return nothing
end


################################################################################
# Wall Constraint
################################################################################

mutable struct Wall
	p1::AbstractVector # initial point of the boundary
	p2::AbstractVector # final point of the boundary
	v::AbstractVector  # vector orthogonal to (p2 - p1) and indicating the forbiden halfspace
end

function add_wall_constraint!(game_con::GameConstraintValues, probsize::ProblemSize,
	walls::AbstractVector{Wall})
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	px = probsize.px

	n_wall = length(walls)
	T = eltype(walls[1].p1)
	x1 = SVector{n_wall,T}([wall.p1[1] for wall in walls])
	y1 = SVector{n_wall,T}([wall.p1[2] for wall in walls])
	x2 = SVector{n_wall,T}([wall.p2[1] for wall in walls])
	y2 = SVector{n_wall,T}([wall.p2[2] for wall in walls])
	xv = SVector{n_wall,T}([wall.v[1] for wall in walls])
	yv = SVector{n_wall,T}([wall.v[2] for wall in walls])

	for i = 1:p
		add_constraint!(
			game_con.state_conlist[i],
			WallConstraint(n, x1, y1, x2, y2, xv, yv, px[i][1], px[i][2]),
			2:N)
		con  = game_con.state_conlist[i].constraints[end]
		inds = game_con.state_conlist[i].inds[end]
		conval = TrajectoryOptimization.ConVal(n,m,con,inds)
		push!(game_con.state_conval[i], conval)
	end
	return nothing
end
