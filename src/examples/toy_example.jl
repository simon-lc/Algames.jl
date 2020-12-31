################################################################################
# Toy Example
################################################################################
T = Float64

# Define the dynamics of the system
p = 3 # Number of players
model = UnicycleGame(p=3) # game with 3 players with unicycle dynamics

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
conlists = [ConstraintList(model.n,model.m,N) for i=1:p] # Empty constraint lists
game_conlist = GameConstraintList(p, conlists)

# Define the initial state of the system
x0 = SVector{model.n,T}([
      0,+0.4,0,0,
      0, 0.0,0,0,
      0,-0.4,0,0])

# Define the Options of the solver
opts = Options()

# Define the game problem
prob = GameProblem(N,dt,x0,model,opts,game_obj,game_conlist)

# # Solve the problem
# solve!(prob)
