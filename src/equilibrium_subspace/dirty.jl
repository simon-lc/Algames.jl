# function subspace_animation(prob::GameProblem)
#     p = prob.probsize.p
#     prob.opts.inner_print = false
#     prob.opts.outer_print = false
#     span = -0.3:0.15:0.2
#     S = vcat([[ exp.(log(10)*[x,y]) for y in span] for x in span]...)
#
#     plt = plot(legend=false)
#
#     anim = @animate for s in S
#         prob.opts.αx_dual = [sqrt(s[1]), sqrt(s[2]), 1.0/(sqrt(s[1]*s[2]))]
#         set_constraint_params!(prob.game_con, prob.opts)
#         newton_solve!(prob)
#         plot_traj!(prob.model, prob.pdtraj.pr, plt=plt)
#         @show mean(abs.(prob.stats.opt_vio[end].vio))
#     end
#     path = joinpath("~/Documents/p3_subspace_uni.gif")
#     gif(anim, path, fps = 10)
#     return nothing
# end
#
# # subspace_animation(prob)
#
#
#
#
# function kkt_residual(prob::GameProblem)
#     residual!(prob, prob.pdtraj)
#     col_res = collision_residual(prob)
#     kkt_res = vcat(prob.core.res, col_res)
#     return kkt_res
# end
#
# function collision_residual(prob::GameProblem)
#     probsize = prob.probsize
#     N = probsize.N
#     p = probsize.p
#     px = probsize.px
#     n_col = Int(p*(p-1)/2)
#     col_res = zeros((N-1)*n_col)
#
#     off = 1
#     for k = 2:N #fix for correct indexing
#         for i = 1:p
#             for j = i+1:p
#                 ind = findfirst(x -> all(x.con.x2 .== px[j]), prob.game_con.state_conval[i])
#                 conval = prob.game_con.state_conval[i][ind]
#                 col_res[off] = conval.vals[k-1][1]
#                 off += 1
#             end
#         end
#     end
#     return col_res
# end
#
#
# function kkt_residual_jacobian(prob::GameProblem)
#     probsize = prob.probsize
#     N = probsize.N
#     p = probsize.p
#     px = probsize.px
#     n_col = Int(p*(p-1)/2)
#     n_λ = p*(p-1)
#     n_res = length(prob.core.res)
#
#     residual_jacobian!(prob, prob.pdtraj)
#     kkt_jac = spzeros(n_res + (N-1)*n_col, n_res + (N-1)*n_λ)
#     kkt_jac[1:n_res, 1:n_res] .= prob.core.jac
#     off = 1
#     for k = 2:N
#         for i = 1:p
#             stamp = stampify(:opt, i, :x, 1, k)
#             vind = prob.core.verti_inds[stamp]
#             for j ∈ setdiff(1:p,i)
#                 ind = findfirst(x -> all(x.con.x2 .== px[j]), prob.game_con.state_conval[i])
#                 conval = prob.game_con.state_conval[i][ind]
#                 hind = n_res + off
#                 kkt_jac[vind, hind:hind] .= conval.jac[k-1]'
#                 off += 1
#             end
#         end
#     end
#
#     off = 1
#     for k = 2:N
#         for i = 1:p
#             hind = prob.core.horiz_inds[:x][1][k]
#             for j = i+1:p
#                 ind = findfirst(x -> all(x.con.x2 .== px[j]), prob.game_con.state_conval[i])
#                 conval = prob.game_con.state_conval[i][ind]
#                 vind = n_res + off
#                 kkt_jac[vind:vind, hind] .= conval.jac[k-1]
#                 off += 1
#             end
#         end
#     end
#     return kkt_jac
# end
#
# function update_λcol!(prob::GameProblem, Δλcol)
#     probsize = prob.probsize
#     N = probsize.N
#     p = probsize.p
#     px = probsize.px
#     off = 1
#     for k = 2:N
#         for i = 1:p
#             for j ∈ setdiff(1:p,i)
#                 ind = findfirst(x -> all(x.con.x2 .== px[j]), prob.game_con.state_conval[i])
#                 conval = prob.game_con.state_conval[i][ind]
#                 # @show off
#                 # @show "before" conval.λ[k-1][1]
#                 if conval.λ[k-1][1] != 0
#                     conval.λ[k-1][1] += Δλcol[off]
#                     conval.λ[k-1][1] = max(0, conval.λ[k-1][1])
#                 end
#                 # @show "after " conval.λ[k-1][1]
#                 off += 1
#             end
#         end
#     end
#     return nothing
# end
#
# function display_nullspace(prob_; M::Int=5, amplitude::T=1e-3, atol::T=1e-10) where {T}
#     kkt_jac = kkt_residual_jacobian(prob_)
#     kkt_jac = Matrix(kkt_jac)
#     n_res = length(prob_.core.res)
#     ns = nullspace(kkt_jac, atol=atol)
#     n_vec = size(ns)[2]
#     nv = ns[:,rand(1:n_vec)]
#     nv = nv./mean(abs.(nv))*amplitude
#     plt = plot(legend=false)
#     anim = @animate for k = 1:M
#         prob = deepcopy(prob_)
#         Δtraj = nv[1:n_res]
#         Δλcol = nv[n_res+1:end]
#         set_traj!(prob.core, prob.Δpdtraj, k*Δtraj)
#         # update_traj!(prob.pdtraj, prob.pdtraj, 1.0, prob.Δpdtraj)
#         update_λcol!(prob, k*Δλcol)
#         # @show Δλcol
#
#         prob.opts.shift = 0
#         prob.opts.α_dual = 0.0
#         prob.opts.αx_dual = zeros(p)
#         prob.opts.dual_reset = false
#         prob.opts.ρ_0 = prob.pen.ρ[1]
#         prob.opts.ρ_max = prob.pen.ρ[1]
#         prob.opts.inner_print = false
#         prob.opts.outer_print = false
#         prob.opts.inner_iter = 20
#         prob.opts.outer_iter = 3
#         prob.opts.reg_0 = 1e-8
#         newton_solve!(prob)
#         residual!(prob)
#         @show mean(abs.(prob.core.res))
#         # @show mean(abs.(Δtraj))
#         # @show mean(abs.(Δλcol))
#         opt = mean(abs.(prob.stats.opt_vio[end].vio))
#         sta = mean(abs.(prob.stats.sta_vio[end].vio))
#         dyn = mean(abs.(prob.stats.dyn_vio[end].vio))
#         println("opt = "*scn(opt)*"  ", opt <= prob.opts.ϵ_opt )
#         println("sta = "*scn(sta)*"  ", opt <= prob.opts.ϵ_opt )
#         println("dyn = "*scn(dyn)*"  ", opt <= prob.opts.ϵ_opt )
#         # plot_violation!(prob.stats)
#         if opt < 1e-5
#             plot_traj!(prob.model, prob.pdtraj.pr, plt=plt)
#         end
#         # plot_violation!(prob.stats)
#     end
#     path = joinpath("~/Documents/nullspace_p3_din.gif")
#     gif(anim, path, fps = 10)
#     return nothing
# end
#
#
# function follow_nullspace(prob; M::Int=5, amplitude::T=1e-3, atol::T=1e-10) where {T}
#
#     plt = plot(legend=false)
#     anim = @animate for k = 1:M
#         Random.seed!(k)
#         kkt_jac = kkt_residual_jacobian(prob)
#         kkt_jac = Matrix(kkt_jac)
#         n_res = length(prob.core.res)
#         ns = nullspace(kkt_jac, atol=atol)
#         n_vec = size(ns)[2]
#         nv = ns[:,10]
#         # nv = ns[:,rand(1:n_vec)]
#         nv = nv./mean(abs.(nv))*amplitude
#
#         Δtraj = nv[1:n_res]
#         Δλcol = nv[n_res+1:end]
#         set_traj!(prob.core, prob.Δpdtraj, Δtraj)
#         update_traj!(prob.pdtraj, prob.pdtraj, 1.0, prob.Δpdtraj)
#         update_λcol!(prob, Δλcol)
#
#         prob.opts.shift = 0
#         prob.opts.α_dual = 0.0
#         prob.opts.αx_dual = zeros(p)
#         prob.opts.dual_reset = false
#         prob.opts.ρ_0 = prob.pen.ρ[1]
#         prob.opts.ρ_max = prob.pen.ρ[1]
#         prob.opts.inner_print = false
#         prob.opts.outer_print = false
#         prob.opts.inner_iter = 20
#         prob.opts.outer_iter = 10
#         prob.opts.reg_0 = 1e-8
#         newton_solve!(prob)
#         residual!(prob)
#         println("***********************")
#         opt = mean(abs.(prob.stats.opt_vio[end].vio))
#         sta = mean(abs.(prob.stats.sta_vio[end].vio))
#         dyn = mean(abs.(prob.stats.dyn_vio[end].vio))
#         println("opt = "*scn(opt)*"  ", opt <= prob.opts.ϵ_opt )
#         println("sta = "*scn(sta)*"  ", opt <= prob.opts.ϵ_opt )
#         println("dyn = "*scn(dyn)*"  ", opt <= prob.opts.ϵ_opt )
#         if opt < 1e-5
#             plot_traj!(prob.model, prob.pdtraj.pr, plt=plt)
#         end
#         # plot_violation!(prob.stats)
#     end
#     path = joinpath("~/Documents/follow_nullspace_p3_uni.gif")
#     gif(anim, path, fps = 20)
#     return nothing
# end
#
# # evaluate!(prob.game_con, prob.pdtraj.pr)
# # collision_residual(prob)
# # kkt_residual(prob)
# # kkt_jac = kkt_residual_jacobian(prob)
# # ns = nullspace(Matrix(kkt_jac), atol=1e-10)
# # nv = ns[:,1]
#
# prob_copy = deepcopy(prob)
# # display_nullspace(prob_copy, M=10, amplitude=1e-2)
# follow_nullspace(prob_copy, M=10, amplitude=1e-4)
#
#
#
#
#
#
#
# function generate_data(n::Int=3, M::Int=10)
#     mean = 10*randn(n)
#     s = randn(n)
#     scale = s*s' + 0.5I
#     data = []
#     for k = 1:M
#         x = mean + scale*randn(n)
#         push!(data, x)
#     end
#     scatter(hcat(data...))
#     return data
# end
#
# function pca(data)
#     plt = plot(aspectratio=1.0)
#     M = length(data)
#     n = length(data[1])
#
#     mean = sum(data)/M
#     data_c = [d .- mean for d in data]
#     cov = 1/M*sum([ d*d' for d in data_c])
#     data_cr = [inv(sqrt(cov))*d for d in data_c]
#     vecs = eigvecs(cov)
#     vals = eigvals(cov)
#     eig = [(vecs[:,i], vals[i]) for i=1:n]
#     sort!(eig, by=x -> x[2], rev=true)
#     for k = 1:n
#         @show round.(eig[k][1], digits=3)
#         @show round(eig[k][2], digits=3)
#     end
#     @show round.(mean, digits=3)
#     @show round.(cov, digits=3)
#
#
#     plt = plot(aspectratio=1.0)
#     scatter!([d[1] for d in data], [d[2] for d in data], color=:blue)
#     scatter!([d[1] for d in data_c], [d[2] for d in data_c], color=:red)
#     scatter!([d[1] for d in data_cr], [d[2] for d in data_cr], color=:orange)
#     for k = 1:n
#         plot!(
#             [0, 4*sqrt(vals[k]).*vecs[1,k]],
#             [0, 4*sqrt(vals[k]).*vecs[2,k]],
#             linewidth=5, color=:black)
#     end
#     display(plt)
#
#
#     data_p = [(d'*eig[1][1])*eig[1][1] + (d'*eig[2][1])*eig[2][1] for d in data]
#     data_p = [(d'*eig[1][1])*eig[1][1] + (d'*eig[2][1])*eig[2][1] for d in data]
#     plt = plot(aspectratio=1.0)
#     scatter!([d[1] for d in data_p], [d[2] for d in data_p], color=:blue)
#     for k = 1:2
#         plot!(
#             [0, 4*sqrt(eig[k][2]).*eig[k][1][1]],
#             [0, 4*sqrt(eig[k][2]).*eig[k][1][2]],
#             linewidth=5, color=:black)
#     end
#     display(plt)
#
#
#     # plt = plot(aspectratio=1.0)
#     # scatter!([d[1] for d in data], [d[2] for d in data], [d[2] for d in data], color=:blue)
#     # scatter!([d[1] for d in data_cr], [d[2] for d in data_c], [d[2] for d in data_c], color=:red)
#     # scatter!([d[1] for d in data_cr], [d[2] for d in data_cr], [d[2] for d in data_cr], color=:orange)
#     # for k = 1:n
#     #     plot!(
#     #         [0. , 4*sqrt(vals[k]).*vecs[1,k]],
#     #         [0. , 4*sqrt(vals[k]).*vecs[2,k]],
#     #         [0. , 4*sqrt(vals[k]).*vecs[3,k]],
#     #         linewidth=10)
#     # end
#     # display(plt)
#     return eig
# end
#
# data = generate_data(3,1000)
# v = pca(data)
# # scatter([d[1] for d in data], [d[2] for d in data], aspectratio=1.0)
# # scatter([d[1] for d in data], [d[2] for d in data], [d[3] for d in data])
#
# s = randn(3)
# sc = s*s'
# vecs = eigvecs(sc)
# vals = eigvals(sc)
# sc*vecs[:,1] - vals[1]*vecs[:,1]
# sc*vecs[:,2] - vals[2]*vecs[:,2]
# sc*vecs[:,3] - vals[3]*vecs[:,3]
#
# norm(vecs[:,1])
