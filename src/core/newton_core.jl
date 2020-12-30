################################################################################
# ProblemSize
################################################################################

mutable struct ProblemSize{SVu,SVx,SVz}
    N::Int
    n::Int
    m::Int
    p::Int
    ni::Vector{Int}
    mi::Vector{Int}
    pu::SVu
    px::SVx
    pz::SVz
end

function ProblemSize(N::Int, model::AbstractGameModel)
    return ProblemSize(
        N,
        model.n,
        model.m,
        model.p,
        model.ni,
        model.mi,
        model.pu,
        model.px,
        model.pz,
        )
end

import Base.==
function (==)(p1::ProblemSize, p2::ProblemSize)
    out = true
    for name in fieldnames(ProblemSize)
        out &= getfield(p1, name) == getfield(p2, name)
    end
    return out
end

################################################################################
# NewtonCore
################################################################################

mutable struct NewtonCore{Vr,Vrt,SMj,SVhi,SVvi,SAr,SAj}
	res::Vr               # residual vector
	res_tmp::Vrt          # holds a temporary copy of the residual vector
	jac::SMj              # residual sparse jacobian
	probsize::ProblemSize # size of the problem
	horiz_inds::SVhi      # indices for each variable in an horizontal block
	verti_inds::SVvi      # indices for each variable in an vertical block
	res_sub::SAr               # Residual views dictionary
	jac_sub::SAj               # Jacobian views dictionary
end

function NewtonCore(probsize::ProblemSize)
	N = probsize.N
	n = probsize.n
	m = probsize.m
	p = probsize.p
	S = n*p*(N-1) + m*(N-1) + n*(N-1)
	res = zeros(S)
	res_tmp = deepcopy(res)
	jac = spzeros(S,S)
	verti_inds = vertical_indices(probsize)
	horiz_inds = horizontal_indices(probsize)
	res_sub = residual_views(res, probsize, verti_inds)
	jac_sub = jacobian_views(jac, probsize, verti_inds, horiz_inds)
	TYPE = typeof.((res, res_tmp, jac, horiz_inds, verti_inds, res_sub, jac_sub))
	return NewtonCore{TYPE...}(res, res_tmp, jac, probsize,
		horiz_inds, verti_inds, res_sub, jac_sub)
end

################################################################################
# Indices
################################################################################

function vertical_indices(probsize::ProblemSize)
	N = probsize.N
	n = probsize.n
	p = probsize.p
	mi = probsize.mi
	verti_inds = Dict()
	off = 0
	for i = 1:p
		verti_inds[Symbol("opt$i")] = Dict()
		verti_inds[Symbol("opt$i")][:x] = Dict()
		verti_inds[Symbol("opt$i")][:u] = Dict()
		for k = 1:N-1
			verti_inds[Symbol("opt$i")][:x][k+1] = SVector{n,Int}(off .+ (1:n))
			off += n
			verti_inds[Symbol("opt$i")][:u][k] = SVector{mi[i],Int}(off .+ (1:mi[i]))
			off += mi[i]
		end
	end
	verti_inds[:dyn] = Dict()
	verti_inds[:dyn][:x] = Dict()
	for k = 1:N-1
		verti_inds[:dyn][:x][k] = SVector{n,Int}(off .+ (1:n))
		off += n
	end
	return verti_inds
end

function horizontal_indices(probsize::ProblemSize)
	N = probsize.N
	n = probsize.n
	p = probsize.p
	mi = probsize.mi

	horiz_inds = Dict()
	horiz_inds[:x] = Dict()
	for i = 1:p
		horiz_inds[Symbol("u$i")] = Dict()
		horiz_inds[Symbol("λ$i")] = Dict()
	end

	off = 0
	for k = 1:N-1

		horiz_inds[:x][k+1] = SVector{n,Int}(off .+ (1:n))
		off += n
		for i = 1:p
			horiz_inds[Symbol("u$i")][k] = SVector{mi[i],Int}(off .+ (1:mi[i]))
			off += mi[i]
		end
		for i = 1:p
			horiz_inds[Symbol("λ$i")][k] = SVector{n,Int}(off .+ (1:n))
			off += n
		end
	end
	return horiz_inds
end

function idx(core::NewtonCore, stamp::Stamp)
	verti = vertical_idx(core, stamp.prob, stamp.n1, stamp.v1)
	horiz = horizontal_idx(core, stamp.n2, stamp.v2)
	return (verti, horiz)
end

function idx(verti_inds::Dict, horiz_inds::Dict, stamp::Stamp)
	verti = verti_inds[stamp.prob][stamp.n1][stamp.v1]
	horiz = horiz_inds[stamp.n2][stamp.v2]
	return (verti, horiz)
end

function horizontal_idx(core::NewtonCore, name::Symbol, v::Int)
	return core.horiz_inds[name][v]
end

function horizontal_idx(horiz_inds::Dict, name::Symbol, v::Int)
	return horiz_inds[name][v]
end

function vertical_idx(core::NewtonCore, prob::Symbol, name::Symbol, v::Int)
	return core.verti_inds[prob][name][v]
end

function vertical_idx(core::NewtonCore, stamp::VStamp)
	return core.verti_inds[stamp.prob][stamp.n1][stamp.v1]
end

function vertical_idx(verti_inds::Dict, stamp::VStamp)
	return verti_inds[stamp.prob][stamp.n1][stamp.v1]
end


################################################################################
# SubArrays
################################################################################

function residual_views(res::SV, probsize::ProblemSize, verti_inds::Dict) where {SV}
	N = probsize.N
	p = probsize.p

	# Residual Views
	res_sub = Dict{VStamp,SubArray}()
	for prob ∈ [:dyn; [Symbol("opt$i") for i=1:p]]
		for n1 in (:x,:u)
			for v1 in 1:N
				stamp = stampify(prob, n1, v1)
				if valid(stamp,N,p)
					res_sub[stamp] = view(res, vertical_idx(verti_inds, stamp))
				end
			end
		end
	end
	return res_sub
end

function jacobian_views(jac::SM, probsize::ProblemSize, verti_inds::Dict, horiz_inds::Dict) where {SM}
	N = probsize.N
	p = probsize.p

	# Jacobian Views
	jac_sub = Dict{Stamp,SubArray}()
	for prob ∈ [:dyn; [Symbol("opt$i") for i=1:p]]
		for n1 in (:x,:u)
			for v1 in 1:N
				for n2 in [:x; [Symbol("u$i") for i=1:p]; [Symbol("λ$i") for i=1:p]]
					for v2 = 1:N #v1-4:v1+4N
						stamp = stampify(prob, n1, v1, n2, v2)
						if valid(stamp,N,p)
							jac_sub[stamp] = view(jac, idx(verti_inds, horiz_inds, stamp)...)
						end
					end
				end
			end
		end
	end
	return jac_sub
end
