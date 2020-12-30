################################################################################
# GameOptions
################################################################################

@with_kw mutable struct Options{T}
    # Options
	"Gauss-newton convergence criterion: tolerance."
	θ::T=1e-2

	# Regularization
	"Regularization of the residual and residual Jacobian."
	regularize::Bool=true

	# "Current Jacobian regularization for each primal-dual variables."
	# reg::Regularizer{T}=Regularizer()

	"Initial Jacobian regularization."
	reg_0::T=1e-3

	# Line search
	"Initial step size."
	α_0::T=1.0

	"Line search increase step."
	α_increase::T=1.2

	"Line search decrease step."
	α_decrease::T=0.5

	"Expected residual improvement."
	β::T=0.01

	"Number of line search iterations."
	ls_iter::Int=25

	# Augmented Lagrangian Penalty
	"Initial augmented Lagrangian penalty."
	ρ_0::T=1.0

	"Fixed augmented Lagrangian penalty on the residual used for the line search trials."
	ρ_trial::T=1.0

	"Penalty increase step."
	ρ_increase::T=10.0

	"Maximum augmented Lagrangian penalty."
	ρ_max::T=1e7

	# Augmented Lagrangian iterations.
	"Outer loop iterations."
	outer_iter::Int=7

	"Inner loop iterations."
	inner_iter::Int=20

	# Problem Scaling
	"Objective function scaling."
	γ::T=1e0

	# MPC
	"Number of time steps simulated with MPC"
	mpc_horizon::Int=20

	"Rate of the upsampling used for simulaion and feedback control in the MPC"
	upsampling::Int=2

	# Printing
	"Displaying the inner step results."
	inner_print::Bool=true

	"Displaying the outer step results."
	outer_print::Bool=true
end

################################################################################
# GameObjective
################################################################################

mutable struct GameObjective
	p::Int
	obj::Vector{Objective}
end

function GameObjective(Q::Vector{DQ}, R::Vector{DR}, xf::Vector{SVx}, uf::Vector{SVu},
	N::Int, model::AbstractGameModel) where {DQ,DR,SVx,SVu}
	T = eltype(xf[1])
	n = model.n
	m = model.m
	p = model.p
	@assert length(Q) == length(R) == length(xf) == length(uf)

	obj = Vector{Objective}(undef, p)
	for i = 1:p
		Qi = Diagonal(SVector{n,T}(expand_vector(diag(Q[i]), model.pz[i], n)))
		Ri = Diagonal(SVector{m,T}(expand_vector(diag(R[i]), model.pu[i], m)))
		Rf = Diagonal(zeros(SVector{m,T}))
		xfi = SVector{n,T}(expand_vector(xf[i], model.pz[i], n))
		ufi = SVector{m,T}(expand_vector(uf[i], model.pu[i], m))
		cost = LQRCost(Qi,Ri,xfi,ufi,checks=false)
		cost_term = LQRCost(Qi,Rf,xfi,checks=false)
		obj[i] = Objective(cost, cost_term, N)
	end
	return GameObjective(p,obj)
end

function expand_vector(v::AbstractVector{T}, inds::AbstractVector{Int}, n::Int) where {T}
	V = zeros(n)
	V[inds] = v
	return V
end

################################################################################
# GameConstraintList
################################################################################

mutable struct GameConstraintList
	p::Int
	conlist::Vector{TrajectoryOptimization.AbstractConstraintSet}
end

function GameConstraintList(conlists::Vector{<:TrajectoryOptimization.AbstractConstraintSet})
	p = length(conlists)
	return GameConstraintList(p, conlists)
end


# mutable struct Residual15{
# 	SVv,SVvt,SMj,SMsj,SVjs,SVs,SVn,
# 	SVhi,SVhip,SVvi,SS,
# 	}
# 	val::SVv              # residual vector
# 	val_tmp::SVvt         # holds a temporary copy of the residual vector
# 	jac::SMj              # residual sparse jacobian
# 	sym_jac::SMsj         # residual sparse symmetric jacobian (twice as large as jac)
# 	jac_scale::Scale13    # scaling of the Jacobian of the residual
# 	stats::Statistics14   # statistics collected during the solve
# 	sizes::SVs            # sizes of the variables that are optimized for in the primal dual step
# 	names::SVn            # names of the variables that are optimized for in the primal dual step
# 	N::Int                # number of time steps in the trajectory
# 	r::Int                # length of the residual vector
# 	h::Int                # horizontal length of the block (uk,γk,bk,qk+1,λo2k,λn3k)
# 	h_primal::Int         # horizontal length of the block (uk,γk,bk,qk+1)
#
# 	horiz_inds::SVhi         # indices for each variable in an horizontal block
# 	horiz_inds_primal::SVhip # indices for each variable in an horizontal block in the primal only solve.
# 	verti_inds::SVvi         # indices for each variable in an vertical block
# 	sub::SS                  # Jacobian views dictionary
#
# 	# tags::Vector{Tag}
# end

	# function Residual15(N::Int, nq::Int, nu::Int, nc::Int, nγ::Int, nb::Int)
	# 	sizes = SVector{6,Int}([nu, nγ, nb, nq, nq, nq])
	# 	sizes_primal = SVector{4,Int}([nu, nγ, nb, nq])
	# 	names = SVector{6,Symbol}([:u, :γ, :b, :q, :λo2, :λn3])
	# 	names_primal = SVector{4,Symbol}([:u, :γ, :b, :q])
	# 	r,h,h_primal = residual_sizes(N,nq,nu,nc,nγ,nb)
	# 	val = zeros(T,r)
	# 	val_tmp = zeros(T,r)
	# 	jac = spzeros(T,r,r)
	# 	sym_jac = spzeros(T,2r,2r) # not the right size it should be something like 2r <- length(v_mask) = vertical_mask(res, opts) + length(h_mask) = horizontal_mask(res, opts)
	# 	jac_scale = Scale13(1.0)
	# 	stats = Statistics14()
	#
	# 	# Horizontal Index
	# 	horiz_inds = Dict()
	# 	for name in names
	# 		horiz_inds[name] = Dict()
	# 	end
	# 	off = 0
	# 	for k = 1:N-1
	# 		horiz_inds[:u][k] = off .+ SVector{nu,Int}(1:nu)
	# 		off += nu
	# 		horiz_inds[:γ][k] = off .+ SVector{nγ,Int}(1:nγ)
	# 		off += nγ
	# 		horiz_inds[:b][k] = off .+ SVector{nb,Int}(1:nb)
	# 		off += nb
	# 		horiz_inds[:q][k+1] = off .+ SVector{nq,Int}(1:nq)
	# 		off += nq
	# 		horiz_inds[:λo2][k] = off .+ SVector{nq,Int}(1:nq)
	# 		off += nq
	# 		horiz_inds[:λn3][k] = off .+ SVector{nq,Int}(1:nq)
	# 		off += nq
	# 	end
	#
	# 	# Horizontal Index
	# 	horiz_inds_primal = Dict()
	# 	for name in names_primal
	# 		horiz_inds_primal[name] = Dict()
	# 	end
	# 	off = 0
	# 	for k = 1:N-1
	# 		horiz_inds_primal[:u][k] = off .+ SVector{nu,Int}(1:nu)
	# 		off += nu
	# 		horiz_inds_primal[:γ][k] = off .+ SVector{nγ,Int}(1:nγ)
	# 		off += nγ
	# 		horiz_inds_primal[:b][k] = off .+ SVector{nb,Int}(1:nb)
	# 		off += nb
	# 		horiz_inds_primal[:q][k+1] = off .+ SVector{nq,Int}(1:nq)
	# 		off += nq
	# 	end
	#
	# 	# Vertical Index
	# 	verti_inds = Dict()
	# 	for prob ∈ (:opt, :mdp, :nf, :dyn)
	# 		verti_inds[prob] = Dict()
	# 	end
	# 	verti_inds[:opt][:q] = Dict()
	# 	verti_inds[:opt][:u] = Dict()
	# 	verti_inds[:nf][:q] = Dict()
	# 	verti_inds[:nf][:γ] = Dict()
	# 	verti_inds[:mdp][:b] = Dict()
	# 	verti_inds[:dyn][:q] = Dict()
	# 	# OPT
	# 	off = 0
	# 	for k = 1:N-1
	# 		verti_inds[:opt][:q][k+1] = off .+ (1:nq)
	# 		off += nq
	# 	end
	# 	for k = 1:N-1
	# 		verti_inds[:opt][:u][k] = off .+ (1:nu)
	# 		off += nu
	# 	end
	# 	# NF
	# 	for k = 1:N-1
	# 		verti_inds[:nf][:q][k+1] = off .+ (1:nq)
	# 		off += nq
	# 	end
	# 	for k = 1:N-1
	# 		verti_inds[:nf][:γ][k] = off .+ (1:nγ)
	# 		off += nγ
	# 	end
	# 	# MDP
	# 	for k = 1:N-1
	# 		verti_inds[:mdp][:b][k] = off .+ (1:nb)
	# 		off += nb
	# 	end
	# 	# DYN
	# 	for k = 1:N-1
	# 		verti_inds[:dyn][:q][k] = off .+ (1:nq)
	# 		off += nq
	# 	end
	#
	# 	# Jacobian Views
	# 	sub = Dict{Stamp12,SubArray}()
	# 	for prob ∈ (:opt, :mdp, :nf, :dyn)
	# 		for n1 in names
	# 			for v1 in 1:N
	# 				for n2 in names
	# 					for v2 = 1:N #v1-4:v1+4N
	# 						stamp = stampify(prob, n1, v1, n2, v2)
	# 						if valid(stamp, :shared, N) # here we use the mode shared to compute the maximum number of views
	# 							v = view(jac, idx(verti_inds, horiz_inds, stamp)...)
	# 							sub[stamp] = view(jac, idx(verti_inds, horiz_inds, stamp)...)
	# 						end
	# 					end
	# 				end
	# 			end
	# 		end
	# 	end
	#
	#
	# 	# Tags
	# 	∇Jopt = [Tag{nq}(:q_1), Tag{nq}(:q), Tag{nu}(:u), Tag{nγ}(:γ), Tag{nb}(:b)]
	# 	∇Jnf  = [Tag{nq}(:q), Tag{nb}(:b), Tag{nq}(:q1)]
	# 	∇Jmdp = [Tag{nγ}(:γ), Tag{nq}(:q1)]
	# 	tags  = [Tag{nq}(:q_1), Tag{nq}(:q), Tag{nu}(:u), Tag{nγ}(:γ), Tag{nb}(:b), Tag{nq}(:q1)]
	#
	#     TYPE = typeof.([
	# 		val, val_tmp, jac, sym_jac, jac_scale, sizes, names,
	# 		horiz_inds, horiz_inds_primal, verti_inds, sub,
	# 		])
	#     return Residual15{TYPE...}(
	#         val,val_tmp,jac,sym_jac,jac_scale,stats,sizes,names,N,r,h,h_primal,
	# 		horiz_inds, horiz_inds_primal, verti_inds, sub,
	# 		∇Jopt, ∇Jnf, ∇Jmdp, tags,
	# 		)
	# end
	#
	#
	# function residual_sizes(N::Int, nq::Int, nu::Int, nc::Int, nγ::Int, nb::Int)
	# 	# length of the residual vector
	# 	r = (N-1)*(3nq + nu + nγ + nb)
	#  	# horizontal length of the block (uk,γk,bk,qk+1,λo2k,λn3k)
	# 	h = 3nq + nu + nγ + nb
	# 	h_primal = nq + nu + nγ + nb
	#     return r,h,h_primal
	# end
	#
	# function idx(res::Residual15, stamp::Stamp12)
	# 	verti = vertical_idx(res, stamp.prob, stamp.n1, stamp.v1)
	# 	horiz = horizontal_idx(res, stamp.n2, stamp.v2)
	# 	return (verti, horiz)
	# end
	#
	# function idx(verti_inds::Dict, horiz_inds::Dict, stamp::Stamp12)
	# 	verti = verti_inds[stamp.prob][stamp.n1][stamp.v1]
	# 	horiz = horiz_inds[stamp.n2][stamp.v2]
	# 	return (verti, horiz)
	# end
	#
	# function horizontal_idx(res::Residual15, name::Symbol, v::Int)
	# 	return res.horiz_inds[name][v]
	# end
	#
	# function horizontal_idx_primal(res::Residual15, name::Symbol, v::Int)
	# 	return res.horiz_inds_primal[name][v]
	# end
	#
	# function vertical_idx(res::Residual15, prob::Symbol, name::Symbol, v::Int)
	# 	return res.verti_inds[prob][name][v]
	# end
	#
	# function vertical_idx(res::Residual15, stamp::VStamp12)
	# 	return res.verti_inds[stamp.prob][stamp.n1][stamp.v1]
	# end
	#
	# function ResidualDistribution13(res::Residual15)
	# 	dist = ResidualDistribution13(0.0)
	# 	N = res.N
	#     for k = 1:N-1
	# 		dist.u_opt  += mean(abs.(res.val[vertical_idx(res, :opt, :u, k)]  ))
	#         dist.q_opt  += mean(abs.(res.val[vertical_idx(res, :opt, :q, k+1)]))
	# 		dist.b_mdp  += mean(abs.(res.val[vertical_idx(res, :mdp, :b, k)]  ))
	# 		dist.γ_nf   += mean(abs.(res.val[vertical_idx(res, :nf,  :γ, k)]  ))
	#         dist.q_nf   += mean(abs.(res.val[vertical_idx(res, :nf,  :q, k+1)]))
	#         dist.dyn    += mean(abs.(res.val[vertical_idx(res, :dyn, :q, k)]  ))
	#     end
	#     dist.u_opt /= N-1
	# 	dist.q_opt /= N-1
	# 	dist.b_mdp /= N-1
	# 	dist.γ_nf  /= N-1
	#     dist.q_nf  /= N-1
	#     dist.dyn   /= N-1
	#     return dist
	# end



################################################################################
# GameProblem
################################################################################

mutable struct GameProblem12{SVx}
    probsize::ProblemSize
    core::NewtonCore
	x0::SVx
    game_obj::GameObjective
    game_conlist::GameConstraintList
    traj::Traj
    opts::Options
	# stats::Statistics
end

function GameProblem12(N::Int, dt::T, x0::SVx, model::AbstractGameModel, opts::Options,
	cost::GameObjective, con::GameConstraintList
	) where {T,SVx}

	probsize = ProblemSize(N,model)
	traj = Trajectory(model.n,model.m,dt,N)
	# stats = Statistics(...)
	TYPE = typeof.((x0))
	return GameProblem12{TYPE...}(probsize, core, x0, game_obj, con, traj, opts, stats)
end
