import pulp as lp
import random
import time
from datetime import timedelta

# n = number of requests
def pdptw_simulation(n):
    ### Data ###
    '''
    Nodes stored in index:
    0 = Start depot node
    1 until 2n = Pickup and Delivery nodes
    2n + 1 = Final depot node
    
    list "node" is representation of actual location nodes along the line (0.0, 4.1, 1.2, etc)
    list "N" is logical abstraction of each node in the "node" list for algorithm input, with value starting from 0 to 2n + 1
    '''
    node = []
    node.append(0.0)
    for i in range(1, (2 * n) + 1):
        node.append(round(10*random.random(), 1))
    node.append(0.0)

    print("Node generated: " + str(node))

    N = list(range(0, 2 * n + 2))

    # Pickup nodes
    P = N[1 : int(len(N) / 2)]

    # Delivery nodes
    D = N[int(len(N) / 2) : len(N) - 1]

    # Pickup and Delivery nodes join list
    PuD = P + D

    '''
    Nodes load:
    Start depot node load = 0
    Pickup nodes load = 1; Destination nodes load = -1
    Final depot node load = 0
    '''
    L = []

    L.append(0)
    
    for i in range(1,len(N) - 1):
        if(i <= n):
            L.append(1)
        else:
            L.append(-1)

    L.append(0)

    # Time windows
    TW = []

    for i in N:
        TW.append((0,len(N) * 10))

    # Vehicle load capacity
    VQ = 6

    # Travel time and cost matrix
    T = []
    C = []
    for i in range(0,len(N)):
        temp = []
        for j in range(0,len(N)):
            #Same node
            if i == j:
                temp.append(0)
            else:
                temp.append(abs(round(node[i]-node[j], 1)))
        T.append(temp)
        C.append(temp)

    ### Constant ###

    # Big M for B
    M_B = 11000 
    #M_B (len(N) * 10) + 1

    # Big M for Q
    M_Q = VQ + 2

    ### ILP Model ###

    # Create pulp model
    pdptw = lp.LpProblem(name="pdptw", sense=lp.LpMinimize)

    ## Variables ##

    # x_i_j variables (arcs of the graph), with integer bounded {0,1} (constraint 12)
    X = [[lp.LpVariable(f'x_{i}_{j}', lowBound=0, upBound=1, cat=lp.LpInteger) for j in range(len(N))] for i in range(len(N))]

    # Time of the vehicle begins service at each node in N
    B = [lp.LpVariable(f'B_{i}', lowBound=0) for i in range(len(N))]

    # Capacity of the vehicle upon leaving each node in N
    Q = [lp.LpVariable(f'Q_{i}', lowBound=0, cat=lp.LpInteger) for i in range(len(N))]

    ## Objective (Constraint 1) ##
    pdptw += lp.lpDot(X, C)
    # pdptw += 1

    ## Constraints ##

    # Prevent same node travel
    pdptw += lp.lpSum(X[i][i] for i in N) == 0

    # Each request is served exactly once (constraints 2 & 3)
    for i in P:
        pdptw += lp.lpSum(X[i][j] for j in N) == 1

    for i in P:
        pdptw += (lp.lpSum(X[i][j] for j in N) - lp.lpSum(X[n + i][j] for j in N)) == 0

    # Start at start depot (constraint 4)
    pdptw += lp.lpSum(X[N[0]][j] for j in N) == 1
        
    # Route (constraint 5)
    for i in PuD: 
        pdptw += (lp.lpSum(X[j][i] for j in N) - lp.lpSum(X[i][j] for j in N)) == 0
        
    # Finish at destination depot (constraint 6)
    pdptw += lp.lpSum(X[i][2 * n + 1] for i in N) == 1

    # Time consistency (constraint 7)
    for i in N:
        for j in N:
            pdptw += B[j] >= (B[i] + T[i][j]) - ((1 - X[i][j]) * M_B)

    # Load consistency (constraint 8)
    for i in N:
        for j in N:
            pdptw += Q[j] >= (Q[i] + L[j]) - ((1 - X[i][j]) * M_Q)

    # Pickup node is visited before the Delivery node (constraint 9)
    for i in P:
        pdptw += B[i] + T[i][n + i] <= B[n + i]

    # Time windows (constraint 10)
    for i in N:
        pdptw += TW[i][0] <= B[i]
        
    for i in N:
        pdptw += B[i] <= TW[i][1]

    # Capacity (constraint 11 modification)
    for i in P:
        pdptw += 1 <= Q[i]
        pdptw += Q[i] <= 6
    
    for i in D:
        pdptw += 0 <= Q[i]
        pdptw += Q[i] <= 5

    # Capacity at starting depot and destination depot
    pdptw += Q[0] == 0
    pdptw += Q[2 * n + 1] == 0
    
    ## ILP solver ##
    starting_time = time.time()
    print("Solving process started...")

    pdptw.solve()

    # Option to run it using Gurobi
    # gurobi_solver = lp.getSolver('GUROBI_CMD')
    # pdptw.solve(gurobi_solver)

    ## ILP result ##

    print(f'Status: {pdptw.status}, {lp.LpStatus[pdptw.status]}')
    print(f'Objective: {pdptw.objective.value()}')

    print("Route taken:")
    start_depot_node = N[0]
    final_depot_node = N[-1]
    current_node = start_depot_node

    print(f'Start route on node {node[0]}. Capacity: {Q[current_node].value()}')
    while current_node != final_depot_node:
        for i in N:
            if X[current_node][i].value() > 0:
                if i in P:
                    print(f'Pickup on node {node[i]} using edge x({node[current_node]}, {node[i]}), route begin at {B[current_node].value()} for a duration & cost of {T[current_node][i]}. Capacity: {Q[i].value()}')
                elif i in D:
                    print(f'Delivery on node {node[i]} using edge x({node[current_node]}, {node[i]}), route begin at {B[current_node].value()} for a duration & cost of {T[current_node][i]}. Capacity: {Q[i].value()}')
                else:
                    print(f'Finish route on node {node[i]} using edge x({node[current_node]}, {node[i]}), route begin at {B[current_node].value()} for a duration & cost of {T[current_node][i]}. Capacity: {Q[i].value()}')
                current_node = i
                break
    
    ending_time = time.time()
    print("Time taken to solve: " + str(timedelta(seconds=ending_time - starting_time)))

# Multiple instance randomizer for subquestion 3, n = number of requests
# min_n = 10
# max_n = 30
# for i in range(18):
#     n = random.randint(min_n, max_n)
#     pdptw_simulation(n)

# Single instance for testing
pdptw_simulation(6)