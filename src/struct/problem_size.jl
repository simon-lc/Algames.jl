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
