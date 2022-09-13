import cplex
from util import *
from addPrimal import *

def initBigM(instance):
    return max(instance._XRange[1], instance._YRange[1]) - min(instance._XRange[0], instance._YRange[0])

def translateSolution(instance, warmStartSolution):
    vx = warmStartSolution[-2]
    vy = warmStartSolution[-1]
    supports = instance.convert(warmStartSolution)
    statusX, xValues = solve_addPrimalX_initial(supports, instance)
    statusY, yValues = solve_addPrimalY_initial(supports, instance)

    py = [xValues[j] for j in range(instance._yNum)]
    px = [yValues[i] for i in range(instance._xNum)]
    slack_x = [xValues[i + 1 + instance._yNum] for i in range(instance._xNum)]
    slack_y = [yValues[j + 1 + instance._xNum] for j in range(instance._yNum)]

    start = py + px + slack_x + slack_y + warmStartSolution[-2:]
    return start

def getOpt(instance, warmStartSolution):
    bigM = initBigM(instance)
    
    ## Variables
    model = cplex.Cplex()

    m_lb = []
    m_ub = []
    m_type = ""
    m_objective = []
    m_variableName = []

    ####### p
    # py -- for x: j
    for j in range(instance._yNum):
        m_lb.append(0.0)
        m_ub.append(1.0)
        m_type += "C"
        m_objective.append(0.0)
        m_variableName.append("px%d" % j)
    # px -- for y: i + instance._yNum
    for i in range(instance._xNum):
        m_lb.append(0.0)
        m_ub.append(1.0)
        m_type += "C"
        m_objective.append(0.0)
        m_variableName.append("py%d" % i)

    ####### slack variable
    # slack x: i + instance._yNum + instance._xNum
    for i in range(instance._xNum):
        m_lb.append(0.0)
        m_ub.append(cplex.infinity)
        m_type += "C"
        m_objective.append(0.0)
        m_variableName.append("sx%d" % i)

    # slack y: j + instance._yNum + instance._xNum + instance._xNum
    for j in range(instance._yNum):
        m_lb.append(0.0)
        m_ub.append(cplex.infinity)
        m_type += "C"
        m_objective.append(0.0)
        m_variableName.append("sy%d" % j)

    ####### binary variable
    # binary x: i + instance._yNum + instance._xNum + instance._xNum + instance._yNum
    for i in range(instance._xNum):
        m_lb.append(0.0)
        m_ub.append(1.0)
        m_type += "B"
        m_objective.append(0.0)
        m_variableName.append("x%d" % i)

    # binary y: j + instance._yNum + instance._xNum + instance._xNum + instance._yNum + instance._xNum
    for j in range(instance._yNum):
        m_lb.append(0.0)
        m_ub.append(1.0)
        m_type += "B"
        m_objective.append(0.0)
        m_variableName.append("y%d" % j)

    ######## Vi
    # x: -2
    m_lb.append(0.0)
    m_ub.append(cplex.infinity)
    m_type += "C"
    m_objective.append(1.0) ## di, coefficients
    m_variableName.append("Vx")

    # y: -1
    m_lb.append(0.0)
    m_ub.append(cplex.infinity)
    m_type += "C"
    m_objective.append(1.0) ## di, coefficients
    m_variableName.append("Vy")

    ############################ Model
    model.objective.set_sense(model.objective.sense.maximize)
    model.variables.add(names=m_variableName, lb=m_lb, ub=m_ub, obj=m_objective, types=m_type)

    ############################ Constraint
    m_matrix = []
    m_rhs = []
    m_constraint_name = []
    m_constraint_senses = []

    index_px = [j for j in range(instance._yNum)]
    index_py = [i + instance._yNum for i in range(instance._xNum)]

    ######## 3b
    # x
    for i in range(instance._xNum):
        values = []
        vIndex = len(m_variableName) - 2
        sIndex = i + instance._yNum + instance._xNum
        
        for j in range(instance._yNum):
            values.append(instance._value[i][j]['x'])
        m_matrix.append(cplex.SparsePair(ind=index_px + [sIndex, vIndex], val=values + [1.0,-1.0]))
        m_rhs.append(0.0)
        m_constraint_senses.append("E")
        m_constraint_name.append("cxE%d" % i)

    # y
    for j in range(instance._yNum):
        values = []
        vIndex = len(m_variableName) - 1
        sIndex = j + instance._yNum + instance._xNum + instance._xNum

        for i in range(instance._xNum):
            values.append(instance._value[i][j]['y'])
        m_matrix.append(cplex.SparsePair(ind=index_py + [sIndex, vIndex], val=values + [1.0,-1.0]))
        m_rhs.append(0.0)
        m_constraint_senses.append("E")
        m_constraint_name.append("cyE%d" % i)


    ######## 3c
    # x
    
    m_matrix.append(cplex.SparsePair(ind=index_px, val=[1.0 for i in index_px]))
    m_rhs.append(1.0)
    m_constraint_senses.append("E")
    m_constraint_name.append("p")

    #y
    
    m_matrix.append(cplex.SparsePair(ind=index_py, val=[1.0 for j in index_py]))
    m_rhs.append(1.0)
    m_constraint_senses.append("E")
    m_constraint_name.append("p")


    ######## 3d
    # px, x
    for i in range(instance._xNum):
        binaryIndex = i + instance._yNum + instance._xNum + instance._xNum + instance._yNum
        pxIndex = i + instance._yNum
        m_matrix.append(cplex.SparsePair(ind=[pxIndex, binaryIndex], val=[1.0, -1.0]))
        m_rhs.append(0.0)
        m_constraint_senses.append("L")
        m_constraint_name.append("cpx%d" % i)

    # py, y
    for j in range(instance._yNum):
        binaryIndex = j + instance._yNum + instance._xNum + instance._xNum + instance._yNum + instance._xNum
        pyIndex = j
        m_matrix.append(cplex.SparsePair(ind=[pyIndex, binaryIndex], val=[1.0, -1.0]))
        m_rhs.append(0.0)
        m_constraint_senses.append("L")
        m_constraint_name.append("cpy%d" % j)

    ######## 3e
    # sx, x
    for i in range(instance._xNum):
        sIndex = i + instance._yNum + instance._xNum
        binaryIndex = i + instance._yNum + instance._xNum + instance._xNum + instance._yNum
        m_matrix.append(cplex.SparsePair(ind=[sIndex, binaryIndex], val=[1.0, bigM]))
        m_rhs.append(bigM)
        m_constraint_senses.append("L")
        m_constraint_name.append("Mx%d" % i)

    # sy, y
    for j in range(instance._yNum):
        sIndex = j + instance._yNum + instance._xNum + instance._xNum
        binaryIndex = j + instance._yNum + instance._xNum + instance._xNum + instance._yNum + instance._xNum
        m_matrix.append(cplex.SparsePair(ind=[sIndex, binaryIndex], val=[1.0, bigM]))
        m_rhs.append(bigM)
        m_constraint_senses.append("L")
        m_constraint_name.append("My%d" % i)

    model.linear_constraints.add(rhs=m_rhs, lin_expr=m_matrix, names=m_constraint_name, senses=m_constraint_senses)
    if warmStartSolution:
        start = translateSolution(instance, warmStartSolution)
        idx = [i for i in range(len(start) - 2)] + [len(m_lb)-2, len(m_lb)-1]
        model.MIP_starts.add(cplex.SparsePair(ind=idx, val=start), model.MIP_starts.effort_level.auto)

    return model