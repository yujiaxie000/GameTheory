import cplex
from util import *
import time as timeCounter


def solve_addPrimalX(supports, instance):
    supportX, supportY, supportX_M, supportY_M = supports
    modelX = cplex.Cplex()
    disableOutput(modelX)
    sx_lb = []
    sx_ub = []
    sx_objective = []
    sx_variableName = []
    cstat=[]
    rstat=[]
    
    for i in range(instance._yNum):
        sx_lb.append(0.0)
        if i in supportY_M:
            sx_ub.append(0.0)
            cstat +=[modelX.start.status.at_lower_bound]
        else:
            sx_ub.append(1.0)
            cstat +=[modelX.start.status.basic]

        sx_objective.append(0.0)
        sx_variableName.append("p%d" % i)
        

    sx_lb.append(0.0) # index instance_yNum
    sx_ub.append(cplex.infinity)
    sx_objective.append(1.0)
    sx_variableName.append("Ux")
    cstat +=[modelX.start.status.basic]

    for i in range(instance._xNum): # slack variable equality
        sx_lb.append(0.0)
        if i in supportX:
            sx_ub.append(0)
            cstat +=[modelX.start.status.at_lower_bound]
        else:
            sx_ub.append(cplex.infinity)
            cstat +=[modelX.start.status.basic]
        sx_objective.append(0.0)
        sx_variableName.append("ixe%d" % i)

    modelX.objective.set_sense(modelX.objective.sense.maximize)
    modelX.variables.add(names=sx_variableName, lb=sx_lb, ub=sx_ub, obj=sx_objective)

    sx_matrix = []
    sx_rhs = []
    sx_constraint_name = []
    sx_constraint_senses = []

    sx_indices = [k for k in range(instance._yNum)]
    sx_matrix.append(cplex.SparsePair(ind=sx_indices, val=[1.0 for i in range(len(sx_indices))]))
    sx_rhs.append(1.0)
    sx_constraint_senses.append("E")
    sx_constraint_name.append("p")
    rstat +=[modelX.start.status.at_lower_bound]

    for curr in range(instance._xNum):
        values = []
        slackIndex = instance._yNum + 1 + curr
        for i in range(instance._yNum):
            values.append(instance._value[curr][i]['x'])
        sx_matrix.append(cplex.SparsePair(ind=sx_indices + [slackIndex, instance._yNum], val=values + [1.0,-1.0]))
        sx_rhs.append(0.0)
        sx_constraint_senses.append("E")
        sx_constraint_name.append("cxE%d" % curr)
        rstat +=[modelX.start.status.at_lower_bound]


    modelX.linear_constraints.add(rhs=sx_rhs, lin_expr=sx_matrix, names=sx_constraint_name, senses=sx_constraint_senses)

    modelX.solve()

    
    if modelX.solution.get_status() == 1:

        return True, modelX.solution.get_objective_value()
    else:
        return False, None

    

def solve_addPrimalY(supports, instance):
    supportX, supportY, supportX_M, supportY_M = supports
    modelY = cplex.Cplex()
    disableOutput(modelY)
    sy_lb = []
    sy_ub = []
    sy_objective = []
    sy_variableName = []
    cstat=[]
    rstat=[]

    for i in range(instance._xNum):
        sy_lb.append(0.0)
        if i in supportX_M:
            sy_ub.append(0.0)
            cstat +=[modelY.start.status.at_lower_bound]

        else:
            sy_ub.append(1.0)
            cstat +=[modelY.start.status.basic]

        sy_objective.append(0.0)
        sy_variableName.append("p%d" % i)

    sy_lb.append(0.0) # index instance._xNum
    sy_ub.append(cplex.infinity)
    sy_objective.append(1.0)
    sy_variableName.append("Uy")
    cstat +=[modelY.start.status.basic]


    for i in range(instance._yNum): # slack variable equality
        sy_lb.append(0.0)
        if i in supportY:
            sy_ub.append(0)
            cstat +=[modelY.start.status.at_lower_bound]

        else:
            sy_ub.append(cplex.infinity)
            cstat +=[modelY.start.status.basic]
            
        sy_objective.append(0.0)
        sy_variableName.append("iye%d" % i)

    modelY.objective.set_sense(modelY.objective.sense.maximize)
    modelY.variables.add(names=sy_variableName, lb=sy_lb, ub=sy_ub, obj=sy_objective)

    sy_matrix = []
    sy_rhs = []
    sy_constraint_name = []
    sy_constraint_senses = []

    sy_indices = [k for k in range(instance._xNum)]
    sy_matrix.append(cplex.SparsePair(ind=sy_indices, val=[1.0 for i in range(len(sy_indices))]))
    sy_rhs.append(1.0)
    sy_constraint_senses.append("E")
    sy_constraint_name.append("p")
    rstat +=[modelY.start.status.at_lower_bound]

    for curr in range(instance._yNum):
        values = []
        slackIndex = instance._xNum + 1 + curr
        for i in range(instance._xNum):
            values.append(instance._value[i][curr]['y'])
        sy_matrix.append(cplex.SparsePair(ind=sy_indices + [slackIndex, instance._xNum], val=values + [1.0,-1.0]))
        sy_rhs.append(0.0)
        sy_constraint_senses.append("E")
        sy_constraint_name.append("cyE%d" % curr)
        rstat +=[modelY.start.status.at_lower_bound]


    modelY.linear_constraints.add(rhs=sy_rhs, lin_expr=sy_matrix, names=sy_constraint_name, senses=sy_constraint_senses)
    
    # modelY.start.set_start(col_status=cstat, row_status=rstat, col_primal=[], row_primal=[], col_dual=[], row_dual=[])

    # modelY.parameters.lpmethod.set(modelY.parameters.lpmethod.values.dual)
    modelY.solve()
    # return modelY.solution.get_objective_value()
    if modelY.solution.get_status() == 1:

        return True, modelY.solution.get_objective_value()
    else:
        return False, None

def solve_addPrimalX_initial(supports, instance):
    supportX, supportY, supportX_M, supportY_M = supports
    modelX = cplex.Cplex()
    disableOutput(modelX)
    sx_lb = []
    sx_ub = []
    sx_objective = []
    sx_variableName = []

    for i in range(instance._yNum):
        sx_lb.append(0.0)
        sx_ub.append(1.0)
        sx_objective.append(0.0)
        sx_variableName.append("p%d" % i)

    sx_lb.append(0.0) # index instance_yNum
    sx_ub.append(cplex.infinity)
    sx_objective.append(1.0)
    sx_variableName.append("Ux")

    for i in range(instance._xNum): # slack variable equality
        sx_lb.append(0.0)
        sx_ub.append(cplex.infinity)
        sx_objective.append(0.0)
        sx_variableName.append("ixe%d" % i)

    modelX.objective.set_sense(modelX.objective.sense.maximize)
    modelX.variables.add(names=sx_variableName, lb=sx_lb, ub=sx_ub, obj=sx_objective)

    sx_matrix = []
    sx_rhs = []
    sx_constraint_name = []
    sx_constraint_senses = []

    sx_indices = [k for k in range(instance._yNum)]
    sx_matrix.append(cplex.SparsePair(ind=sx_indices, val=[1.0 for i in range(len(sx_indices))]))
    sx_rhs.append(1.0)
    sx_constraint_senses.append("E")
    sx_constraint_name.append("p")

    for curr in range(instance._xNum):
        values = []
        slackIndex = instance._yNum + 1 + curr
        for i in range(instance._yNum):
            values.append(instance._value[curr][i]['x'])
        sx_matrix.append(cplex.SparsePair(ind=sx_indices + [slackIndex, instance._yNum], val=values + [1.0,-1.0]))
        sx_rhs.append(0.0)
        sx_constraint_senses.append("E")
        sx_constraint_name.append("cxE%d" % curr)


    for curr in supportX:
        identifierE = instance._yNum + 1 + curr
        sx_matrix.append(cplex.SparsePair(ind=[identifierE], val=[1.0]))
        sx_rhs.append(0.0)
        sx_constraint_senses.append("E")
        sx_constraint_name.append("rxE%d" % curr)

    for curr in supportX_M:
        identifierE = instance._yNum + 1 + curr
        sx_matrix.append(cplex.SparsePair(ind=[identifierE], val=[1.0]))
        sx_rhs.append(0.0)
        sx_constraint_senses.append("G")
        sx_constraint_name.append("rxL%d" % curr)

    for curr in supportY_M:
        sx_matrix.append(cplex.SparsePair(ind=[curr], val=[1.0]))
        sx_rhs.append(0.0)
        sx_constraint_senses.append("E")
        sx_constraint_name.append("p%d" % curr)

    #print(sx_matrix)
    modelX.linear_constraints.add(rhs=sx_rhs, lin_expr=sx_matrix, names=sx_constraint_name, senses=sx_constraint_senses)
    modelX.solve()
    if modelX.solution.get_status() == 1:

        return True, modelX.solution.get_values()
    else:
        return False, None

def solve_addPrimalY_initial(supports, instance):
    supportX, supportY, supportX_M, supportY_M = supports
    modelY = cplex.Cplex()
    disableOutput(modelY)
    sy_lb = []
    sy_ub = []
    sy_objective = []
    sy_variableName = []

    for i in range(instance._xNum):
        sy_lb.append(0.0)
        sy_ub.append(1.0)
        sy_objective.append(0.0)
        sy_variableName.append("p%d" % i)

    sy_lb.append(0.0) # index instance._xNum
    sy_ub.append(cplex.infinity)
    sy_objective.append(1.0)
    sy_variableName.append("Uy")

    for i in range(instance._yNum): # slack variable equality
        sy_lb.append(0.0)
        sy_ub.append(cplex.infinity)
        sy_objective.append(0.0)
        sy_variableName.append("iye%d" % i)

    modelY.objective.set_sense(modelY.objective.sense.maximize)
    modelY.variables.add(names=sy_variableName, lb=sy_lb, ub=sy_ub, obj=sy_objective)

    sy_matrix = []
    sy_rhs = []
    sy_constraint_name = []
    sy_constraint_senses = []

    sy_indices = [k for k in range(instance._xNum)]
    sy_matrix.append(cplex.SparsePair(ind=sy_indices, val=[1.0 for i in range(len(sy_indices))]))
    sy_rhs.append(1.0)
    sy_constraint_senses.append("E")
    sy_constraint_name.append("p")

    for curr in range(instance._yNum):
        values = []
        slackIndex = instance._xNum + 1 + curr
        for i in range(instance._xNum):
            values.append(instance._value[i][curr]['y'])
        sy_matrix.append(cplex.SparsePair(ind=sy_indices + [slackIndex, instance._xNum], val=values + [1.0,-1.0]))
        sy_rhs.append(0.0)
        sy_constraint_senses.append("E")
        sy_constraint_name.append("cyE%d" % curr)

    for curr in supportY:
        identifierE = instance._xNum + 1 + curr
        sy_matrix.append(cplex.SparsePair(ind=[identifierE], val=[1.0]))
        sy_rhs.append(0.0)
        sy_constraint_senses.append("E")
        sy_constraint_name.append("ryE%d" % curr)

    for curr in supportY_M:
        identifierE = instance._xNum + 1 + curr
        sy_matrix.append(cplex.SparsePair(ind=[identifierE], val=[1.0]))
        sy_rhs.append(0.0)
        sy_constraint_senses.append("G")
        sy_constraint_name.append("ryL%d" % curr)

    for curr in supportX_M:
        sy_matrix.append(cplex.SparsePair(ind=[curr], val=[1.0]))
        sy_rhs.append(0.0)
        sy_constraint_senses.append("E")
        sy_constraint_name.append("p%d" % curr)

    modelY.linear_constraints.add(rhs=sy_rhs, lin_expr=sy_matrix, names=sy_constraint_name, senses=sy_constraint_senses)
    modelY.solve()
    # return modelY.solution.get_objective_value()
    if modelY.solution.get_status() == 1:

        return True, modelY.solution.get_values()
    else:
        return False, None

