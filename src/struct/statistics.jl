################################################################################
# Statistics
################################################################################

mutable struct Statistics#{SVj,SVv,SVd}
	iter::Int
	# obj::SVj
	# vio::SVv
	# dist::SVd
end

function Statistics()
	iter = 0
	# obj = Vector{ObjectiveStatistics}()
	# vio = Vector{ConstraintViolation}()
	# dist = Vector{ResidualDistribution}()
	# TYPE = typeof.([vio, dist])
	# return Statistics{TYPE...}(iter, vio, dist)
	return Statistics(iter)
end

function record!(stats::Statistics)#, obj::ObjtectiveStatistics, vio::ConstraintViolation, dist::ResidualDistribution)
	stats.iter += 1
	# push!(stats.obj, obj)
	# push!(stats.vio, vio)
	# push!(stats.dist, dist)
	return nothing
end

function reset!(stats::Statistics)
	stats_ = Statistics()
	for name in fieldnames(Statistics)
		setfield!(stats, name, getfield(stats_, name))
	end
	return nothing
end
