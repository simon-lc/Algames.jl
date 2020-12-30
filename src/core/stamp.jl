################################################################################
# Stamps
################################################################################

################################################################################
# Type Definition
################################################################################

abstract type AbstractStamp
end

mutable struct Stamp <: AbstractStamp
	prob::Symbol # name of the problem
	n1::Symbol   # name of the variable
	v1::Int      # value of the variable
	n2::Symbol   # name of the variable
	v2::Int      # value of the variable
end

mutable struct VStamp <: AbstractStamp
	prob::Symbol # name of the problem
	n1::Symbol   # name of the variable
	v1::Int      # value of the variable
end

################################################################################
# Base functions
################################################################################

import Base.hash
import Base.==
import Base.isequal

Base.hash(stamp::Stamp, h::UInt)  = hash(stamp.prob, hash(stamp.n1, hash(stamp.v1, hash(stamp.n2, hash(stamp.v2, h)))))
Base.hash(stamp::VStamp, h::UInt) = hash(stamp.prob, hash(stamp.n1, hash(stamp.v1, h)))

function (==)(stamp1::AbstractStamp, stamp2::AbstractStamp)
    out = true
    T1 = typeof(stamp1)
    T2 = typeof(stamp2)
    T1 == T2 ? nothing : return false
    for name in fieldnames(T1)
        out &= getfield(stamp1, name) == getfield(stamp2, name)
    end
    return out
end

function (==)(stamp1::Stamp, stamp2::Stamp)
    out = true
	for name in Stamp.name.names
        out &= getfield(stamp1, name) == getfield(stamp2, name)
    end
    return out
end

function (==)(stamp1::VStamp, stamp2::VStamp)
    out = true
	for name in Stamp.name.names
        out &= getfield(stamp1, name) == getfield(stamp2, name)
    end
    return out
end

Base.isequal(stamp1::Stamp, stamp2::Stamp) = stamp1 == stamp2
Base.isequal(stamp1::VStamp, stamp2::VStamp) = stamp1 == stamp2

################################################################################
# Methods
################################################################################

function Stamp()
	return Stamp(:x, :x, 0, :x, 0)
end

function VStamp()
	return VStamp(:x, :x, 0)
end

function stampify(name::Symbol, step::Int)
	if name == :x_1
		name = :x
		step -= 1
	elseif name == :x1
		name = :x
		step += 1
	end
	return name, step
end

function stampify(prob::Symbol, name_i::Symbol, step_i::Int, name_j::Symbol, step_j::Int)
	name_i, step_i = stampify(name_i, step_i)
	name_j, step_j = stampify(name_j, step_j)
	return Stamp(prob, name_i, step_i, name_j, step_j)
end

function stampify!(stamp::Stamp, prob::Symbol, name_i::Symbol, step_i::Int, name_j::Symbol, step_j::Int)
	name_i, step_i = stampify(name_i, step_i)
	name_j, step_j = stampify(name_j, step_j)
	stamp.prob = prob
	stamp.n1 = name_i
	stamp.v1 = step_i
	stamp.n2 = name_j
	stamp.v2 = step_j
	return nothing
end

function stampify(prob::Symbol, name_i::Symbol, step_i::Int)
	name_i, step_i = stampify(name_i, step_i)
	return VStamp(prob, name_i, step_i)
end

function stampify!(stamp::VStamp, prob::Symbol, name_i::Symbol, step_i::Int)
	name_i, step_i = stampify(name_i, step_i)
	stamp.prob = prob
	stamp.n1 = name_i
	stamp.v1 = step_i
	return nothing
end

################################################################################
# Stamp Validity
################################################################################

function valid(s::Stamp, N::Int, p::Int)
	return valid(s.prob, s.n1, s.v1, s.n2, s.v2, N, p)
end

function valid(prob::Symbol, n1::Symbol, v1::Int, n2::Symbol, v2::Int, N::Int, p::Int)
	vs = VStamp(prob, n1, v1)
	b1 = valid(vs,N,p) # stamp 1 is valid
	b2 = false # stamp 2 is valid

	for i = 1:p
		if prob == Symbol("opt$i")
			for j = 1:p
				if n2 == Symbol("u$j") && v2 ∈ (1:N-1) # uj1, ...N-1
					b2 = true
				end
			end
			if n2 == Symbol("λ$i") && v2 ∈ (1:N-1) # λi1 ...N-1
				b2 = true
			elseif n2 == :x && v2 ∈ (2:N) # x2...xN
				b2 = true
			end
		end
	end
	if prob == :dyn
		for j = 1:p
			if n2 == Symbol("u$j") && v2 ∈ (1:N-1) # uj1, ...N-1
				b2 = true
			end
		end
		if n2 == :x && v2 ∈ (2:N) # x2...xN
			b2 = true
		end
	end
	return b1 && b2
end

function valid(s::VStamp, N::Int, p::Int)
	return valid(s.prob, s.n1, s.v1, N, p)
end

function valid(prob::Symbol, n1::Symbol, v1::Int, N::Int, p::Int)
	b1 = false # stamp 1 is valid
	for i = 1:p
		if prob == Symbol("opt$i")
			if n1 == :u && v1 ∈ (1:N-1) # u1...uN-1
				b1 = true
			elseif n1 == :x && v1 ∈ (2:N) # x2...xN
				b1 = true
			end
		end
	end
	if prob == :dyn
		if n1 == :x && v1 ∈ (1:N-1) # x1...xN-1
			b1 = true
		end
	end
	return b1
end




# stamp = Stamp(:opt, :u, 1, :q_1, 2)
# stamp = stampify(:opt, :q_1, 3, :u, 1)
# valid(stamp, opts_, N)
# opts_.mode = :shared
