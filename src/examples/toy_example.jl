################################################################################
# Toy Example
################################################################################
T = Float64

# Define the dynamics of the system
p = 3 # Number of players
model = UnicycleGame(p=3) # game with 3 players with unicycle dynamics
n = model.n
m = model.m

# Define the horizon of the problem
N = 10 # N time steps
dt = 0.1 # each step lasts 0.1 second
probsize = ProblemSize(N,model) # Structure holding the relevant sizes of the problem

# Define the objective of each player
# We use a LQR cost
Q = [Diagonal(10*ones(SVector{model.ni[i],T})) for i=1:p] # Quadratic state cost
R = [Diagonal(ones(SVector{model.mi[i],T})) for i=1:p] # Quadratic control cost
# Desrired state
xf = [SVector{model.ni[1],T}([2,+0.4,0,0]),
      SVector{model.ni[2],T}([2, 0.0,0,0]),
      SVector{model.ni[3],T}([2,-0.4,0,0])]
# Desired control
uf = [zeros(SVector{model.mi[i],T}) for i=1:p]
# Objectives of the game
game_obj = GameObjective(Q,R,xf,uf,N,model)

# Define the constraints that each player must respect
game_con = GameConstraintValues(probsize)
# Add collision avoidance
radius = 0.1
add_collision_avoidance!(game_con, probsize, radius)
# Add control bounds
u_max =  100*ones(SVector{m,T})
u_min = -100*ones(SVector{m,T})
add_control_bound!(game_con, probsize, u_max, u_min)
# Add wall constraint
walls = [Wall([0.,0.], [1.,0.], [0.,1.])]
add_wall_constraint!(game_con, probsize, walls)
# Add circle constraint
xc = [1., 2., 3.]
yc = [1., 2., 3.]
radius = [0.1, 0.2, 0.3]
add_circle_constraint!(game_con, probsize, xc, yc, radius)

# Define the initial state of the system
x0 = SVector{model.n,T}([
    0,+0.4,0,0,
    0, 0.0,0,0,
    0,-0.4,0,0])

# Define the Options of the solver
opts = Options()

# Define the game problem
prob = GameProblem(N,dt,x0,model,opts,game_obj,game_con)


# Solve the problem
newton_solve!(prob)



function conn(prob::GameProblem)
    N = prob.probsize.N
    n = prob.probsize.n
    p = prob.probsize.p
    game_con = prob.game_con
    pdtraj = prob.pdtraj
    core = prob.core
    jac_sub = core.jac_sub

    stamp = Stamp()
    p_max = maximum(vcat(game_con.control_conlist.p,
        [game_con.state_conlist[i].p for i=1:game_con.p]...)) # the size of the largest constraint
    ∇c = zeros(MMatrix{p_max,n,T,p_max*n})
    for i = 1:p
        conlist = game_con.state_conlist[i]
        for j = 1:length(conlist.inds)
            con = conlist.constraints[j]
            for k in conlist.inds[j]
                stampify!(stamp, :opt, i, :x, 1, k, :x, 1, k)
                zk = pdtraj.pr[k]
                valid(stamp,N,p) ? add_state_constraint_penalty!(jac_sub[stamp], ∇c, con, zk) : nothing
            end
        end
    end

    conlist = game_con.control_conlist
    for j = 1:length(conlist.inds)
        con = conlist.constraints[j]
        for k in conlist.inds[j]
            zk = pdtraj.pr[k]
            add_control_constraint_penalty!(jac_sub, ∇c, con, zk, probsize, k)
        end
    end
    return nothing
end

function add_state_constraint_penalty!(v::SubArray, ∇c::MMatrix,
    con::TrajectoryOptimization.StateConstraint{P}, zk::AbstractKnotPoint)
    ∇c .*= 0.0 # reset the constraint Jacobian
    TrajectoryOptimization.jacobian!(∇c[1:P,:], con, zk)
    # Iρ = con_val.Iρ
    add2sub(v, ∇c'*(Iρ*∇c))
    return nothing
end

function add_control_constraint_penalty!(jac_sub::Dict, ∇c::MMatrix,
    con::TrajectoryOptimization.ControlConstraint{P}, zk::AbstractKnotPoint,
    probsize::ProblemSize, k::Int)
    N = probsize.N
    p = probsize.p
    pu = probsize.pu

    ∇c .*= 0.0 # reset the constraint Jacobian
    TrajectoryOptimization.jacobian!(∇c[1:P,:], con, zk)
    # Iρ = con_val.Iρ
    pen = ∇c'*(Iρ*∇c)

    for i = 1:p
        stampify!(stamp, :opt, i, :u, i, k, :u, i, k)
        valid(stamp,N,p) ? add2sub(jac_sub[stamp], pen[pu[i],pu[i]]) : nothing
    end
    return nothing
end




conn(prob)
p_max = maximum(vcat([prob.game_con.state_conlist[i].p for i=1:prob.game_con.p]...))

prob.game_con.state_conlist[1]
con  = prob.game_con.state_conlist[1].constraints[1]
inds = prob.game_con.state_conlist[1].inds[1]
TrajectoryOptimization.ConVal(model.n, model.m, con, inds)

ControlBoundConstraint(m,u_max=u_max,u_min=u_min)
