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
	# probsize::ProblemSize # size of the problem
	horiz_inds::SVhi      # indices for each variable in an horizontal block
	verti_inds::SVvi      # indices for each variable in an vertical block
	res_sub::SAr               # Residual views dictionary
	jac_sub::SAj               # Jacobian views dictionary
end

function NewtonCore(probsize::ProblemSize)
	res = zeros()
	res_tmp = deepcopy(res)
	# jac =
	verti_inds = vertical_indices(probsize)
	horiz_inds = horizintal_indices(probsize)
	return NewtonCore{TYPE...}(res, res_tmp, jac, probsize,
		horiz_inds, verti_inds, sub)
end

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
			verti_inds[Symbol("opt$i")][:x][k] = SVector{n,Int}(off .+ (1:n))
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

		horiz_inds[:x][k] = SVector{n,Int}(off .+ (1:n))
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
