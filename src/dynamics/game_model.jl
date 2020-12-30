abstract type AbstractGameModel
end

function Base.size(model::AbstractGameModel)
	return model.n, model.m, model.pu, model.p
end
