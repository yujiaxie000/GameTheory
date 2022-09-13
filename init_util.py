import numpy as np
import itertools as it
import cplex
from cplex.exceptions import CplexError
import time as timeCounter

def preprocessMaster(instance):
    Ax = np.arange(instance._xNum)
    Ax_pairs = list(it.combinations(Ax, 2))
    Ay = np.arange(instance._yNum)

    maxEq = 0.0
    maxSupport = None
    maxObj = []

    for x1, x2 in Ax_pairs:
        supportX = [int(x1), int(x2)] 
        supportX_M = [int(i) for i in Ax if i != x1 and i != x2]
        nonDominated = instance.get_nonDominated([x1, x2], 'x')
        Ay_pairs = list(it.combinations(nonDominated, 2))
        for y1, y2 in Ay_pairs:
            supportY = [int(y1), int(y2)]
            supportY_M = [int(j) for j in Ay if j != y1 and j != y2]


            supports = supportX, supportY, supportX_M, supportY_M
            result, objX, objY = quickSolver(supports, instance, nonDominated)

            if result:
                if objX + objY > maxEq:
                    maxEq = objX + objY
                    maxSupport = supports
                    maxObj = [objX, objY]
    if maxSupport:
        supportX, supportY, supportX_M, supportY_M = maxSupport
        maxSupport = supportX + [j + instance._xNum for j in supportY]
    return maxEq, maxSupport, maxObj

def quickSolver(supports, instance, nonDominated):
    (i1, i2), (j1, j2), supportX_M, supportY_M = supports
    utility = instance._value

    py1 = (utility[i2][j2]['x'] - utility[i1][j2]['x'])/(utility[i2][j2]['x'] - utility[i1][j2]['x'] + utility[i1][j1]['x'] - utility[i2][j1]['x'])
    py2 = 1.0 - py1
    px1 = (utility[i2][j2]['y'] - utility[i2][j1]['y'])/(utility[i2][j2]['y'] - utility[i1][j2]['y'] + utility[i1][j1]['y'] - utility[i2][j1]['y'])
    px2 = 1.0 - px1

    v1 = utility[i1][j1]['x'] * py1 + utility[i1][j2]['x'] * py2
    v2 = utility[i1][j1]['y'] * px1 + utility[i2][j1]['y'] * px2

    if px1 < 0 or px2 < 0 or py1 < 0 or py2 < 0:
        return False, None, None

    for j in nonDominated:
        valueY = utility[i1][j]['y'] * px1 + utility[i2][j]['y'] * px2
        if valueY > v2:
            return False, None, None

    for i in supportX_M:
        valueX = utility[i][j1]['x'] * py1 + utility[i][j2]['x'] * py2
        if valueX > v1:
            return False, None, None

    return True, v1, v2


def initMaster(masterP, instance):
    numTotalVar = instance._xNum + instance._yNum
    c_lb = [0.0 for i in range(numTotalVar)] + [-cplex.infinity, -cplex.infinity]
    c_ub = [1.0 for i in range(numTotalVar)] + [cplex.infinity, cplex.infinity]
    c_type = "B" * numTotalVar +  "CC"
    c_objective = [0.0 for i in range(numTotalVar)] + [1.0, 1.0]
    c_variableName = ["x%d" % i for i in range(instance._xNum)] + ["y%d" % j for j in range(instance._yNum)] + ["zx", "zy"]

    c_constraints = []

    c_constraints.append(cplex.SparsePair(ind=[i for i in range(instance._xNum)], val=[1.0 for i in range(instance._xNum)]))
    c_constraints.append(cplex.SparsePair(ind=[j + instance._xNum for j in range(instance._yNum)], val=[1.0 for j in range(instance._yNum)]))



    zx_idx = instance._xNum + instance._yNum
    zy_idx = zx_idx + 1
    c_constraints.append(cplex.SparsePair(ind=[zx_idx], val=[1.0]))
    c_constraints.append(cplex.SparsePair(ind=[zy_idx], val=[1.0]))
    c_constraints.append(cplex.SparsePair(ind=[zx_idx, zy_idx], val=[1.0, 1.0]))

    c_rhs = [1.0, 1.0, instance._maxX, instance._maxY, instance._maxV]
    c_names = ["initX", "initY", "V1", "V2", "Vt"]
    c_senses = ["G", "G", "L", "L", "L"]
    

    masterP.variables.add(names=c_variableName, lb=c_lb, ub=c_ub, types=c_type, obj=c_objective)
    masterP.objective.set_sense(masterP.objective.sense.maximize)
    masterP.linear_constraints.add(lin_expr=c_constraints, rhs=c_rhs, names=c_names, senses=c_senses)
    instanceXMax, instanceYMax, instanceMaxIdx = instance.getMaxEqlm()

    start_time = timeCounter.time()
    processType = True
    warmStartSolution = None
    if instanceMaxIdx:
        masterP.MIP_starts.add(cplex.SparsePair(ind=[int(instanceMaxIdx[0]), int(instanceMaxIdx[1] + instance._xNum), int(zx_idx), int(zy_idx)], val=[1.0, 1.0, instanceXMax, instanceYMax]), masterP.MIP_starts.effort_level.auto)
        warmStartSolution = [0.0 for i in range(len(c_lb))]
        warmStartSolution[int(instanceMaxIdx[0])] = 1.0
        warmStartSolution[int(instanceMaxIdx[1] + instance._xNum)] = 1.0
        warmStartSolution[int(zx_idx)] = instanceXMax
        warmStartSolution[int(zy_idx)] = instanceYMax
    else: 
        
        maxEq, maxSupport, maxObj = preprocessMaster(instance)

        if maxSupport:
            masterP.MIP_starts.add(cplex.SparsePair(ind=maxSupport + [int(zx_idx), int(zy_idx)], val=[1.0] * 4 + maxObj), masterP.MIP_starts.effort_level.auto)
            warmStartSolution = [0.0 for i in range(len(c_lb))]
            for i in maxSupport:
                warmStartSolution[i] = 1.0
            warmStartSolution[int(zx_idx)] = maxObj[0]
            warmStartSolution[int(zy_idx)] = maxObj[1]
        processType = False
    preprocessTime = timeCounter.time() - start_time
    return masterP, preprocessTime, processType, warmStartSolution

def initMaster_Benders(masterP, instance, warmStartSolution):
    numTotalVar = instance._xNum + instance._yNum
    c_lb = [0.0 for i in range(numTotalVar)] + [-cplex.infinity, -cplex.infinity]
    c_ub = [1.0 for i in range(numTotalVar)] + [cplex.infinity, cplex.infinity]
    c_type = "B" * numTotalVar +  "CC"
    c_objective = [0.0 for i in range(numTotalVar)] + [1.0, 1.0]
    c_variableName = ["x%d" % i for i in range(instance._xNum)] + ["y%d" % j for j in range(instance._yNum)] + ["zx", "zy"]
    zx_idx = instance._xNum + instance._yNum
    zy_idx = zx_idx + 1

    c_constraints = []

    c_constraints.append(cplex.SparsePair(ind=[i for i in range(instance._xNum)], val=[1.0 for i in range(instance._xNum)]))
    c_constraints.append(cplex.SparsePair(ind=[j + instance._xNum for j in range(instance._yNum)], val=[1.0 for j in range(instance._yNum)]))
    c_constraints.append(cplex.SparsePair(ind=[zx_idx], val=[1.0]))
    c_constraints.append(cplex.SparsePair(ind=[zy_idx], val=[1.0]))
    c_constraints.append(cplex.SparsePair(ind=[zx_idx, zy_idx], val=[1.0, 1.0]))

    c_rhs = [1.0, 1.0, instance._maxX, instance._maxY, instance._maxV]
    c_names = ["initX", "initY", "V1", "V2", "Vt"]
    c_senses = ["G", "G", "L", "L", "L"]

    masterP.variables.add(names=c_variableName, lb=c_lb, ub=c_ub, types=c_type, obj=c_objective)
    masterP.objective.set_sense(masterP.objective.sense.maximize)
    masterP.linear_constraints.add(lin_expr=c_constraints, rhs=c_rhs, names=c_names, senses=c_senses)

    if warmStartSolution:
        indices = [i for i in range(len(c_lb))]
        masterP.MIP_starts.add(cplex.SparsePair(ind=indices, val=warmStartSolution), masterP.MIP_starts.effort_level.no_check) 
    return masterP