import cplex
from cplex.exceptions import CplexError
from util import *
import numpy as np
import time as timeCounter


class GetMax():
    def __init__(self, instance, threadsNum):
        self._instance = instance
        self._threadsNum = threadsNum
        self._model51X, self._model51Y = self.init51('x'), self.init51('y')
        self._model52, self._start, self._dicts = self.init52()
        self._counter = 0.0

    def get_counter(self):
        return self._counter

    def get_fixed(self, mode, value, ub, lb):
        include = []
        exclude = []

        if mode == "x":
            x_ub = [ub[i] for i in range(self._instance._xNum)]
            x_lb = [lb[i] for i in range(self._instance._xNum)]
            for i in range(len(x_ub)):
                if abs(x_ub[i] - value) < 0.0001 and abs(x_lb[i] - value) < 0.0001:
                    include.append(i)
                else:
                    exclude.append(i)
        else:
            y_ub = [ub[j] for j in range(self._instance._xNum, self._instance._xNum + self._instance._yNum, 1)]
            y_lb = [lb[j] for j in range(self._instance._xNum, self._instance._xNum + self._instance._yNum, 1)]
            for j in range(len(y_ub)):
                if abs(y_ub[j] - value) < 0.0001 and abs(y_lb[j] - value) < 0.0001:
                    include.append(j)
                else:
                    exclude.append(j)

        return include, exclude

    def init51(self, mode):
        model = cplex.Cplex()
        model.parameters.threads.set(self._threadsNum)
        disableOutput(model)

        v_lb = []
        v_ub = []
        v_obj = []
        v_varName = []

        v_constraints = []
        v_rhs = []
        v_names = []
        v_senses = []

        p_idx = []

        if mode == "x":
            for j in range(self._instance._yNum):
                v_lb.append(0.0)
                v_ub.append(1.0)
                v_obj.append(0.0)
                v_varName.append("py%d" % j)
                p_idx.append(j)

        else:
            for i in range(self._instance._xNum):
                v_lb.append(0.0)
                v_ub.append(1.0)
                v_obj.append(0.0)
                v_varName.append("px%d" % i)
                p_idx.append(i)

        v_lb.append(0.0)
        v_ub.append(cplex.infinity)
        v_obj.append(1.0)
        v_varName.append("v%s" % mode)

        v_idx = len(v_lb) - 1

        # 15c
        if mode == "x":
            for i in range(self._instance._xNum):
                values = [self._instance._value[i][j]['x'] for j in range(self._instance._yNum)]
                values.append(-1.0)
                v_constraints.append(cplex.SparsePair(ind=p_idx + [v_idx], val=values))
                v_rhs.append(0.0)
                v_names.append("15cx%d"%i)
                v_senses.append("L")
        else:
            for j in range(self._instance._yNum):
                values = [self._instance._value[i][j]['y'] for i in range(self._instance._xNum)]
                values.append(-1.0)
                v_constraints.append(cplex.SparsePair(ind=p_idx + [v_idx], val=values))
                v_rhs.append(0.0)
                v_names.append("15cy%d"%j)
                v_senses.append("L")

        # 15d
        v_constraints.append(cplex.SparsePair(ind=p_idx, val=[1.0 for i in p_idx]))
        v_rhs.append(1.0)
        v_names.append("p%s" % mode)
        v_senses.append("E")

        model.variables.add(names=v_varName, lb=v_lb, ub=v_ub, obj=v_obj)
        model.objective.set_sense(model.objective.sense.maximize)
        model.linear_constraints.add(lin_expr=v_constraints, rhs=v_rhs, names=v_names, senses=v_senses)

        return model

    def getV_51(self, ub, lb):
        return self.getV_helper(ub, lb, 'x'), self.getV_helper(ub, lb, 'y')

    def getV_helper(self, ub, lb, mode):
        if mode == "x":
            model = cplex.Cplex(self._model51X)
            disableOutput(model)

            includeX, excludeX = self.get_fixed('x', 1.0, ub, lb)
            if not includeX:
                return -1.0

            includeY, excludeY = self.get_fixed('y', 0.0, ub, lb)
            
            # 15f
            if includeY:
                new_ub = [("py%d" % j, 0.0) for j in includeY]
                model.variables.set_upper_bounds(new_ub)

            # 15b
            new_senses = [("15cx%d" % i, "E") for i in includeX]
            model.linear_constraints.set_senses(new_senses)

        else:
            model = cplex.Cplex(self._model51Y)
            disableOutput(model)

            includeY, excludeY = self.get_fixed('y', 1.0, ub, lb)
            if not includeY:
                return -1.0

            includeX, excludeX = self.get_fixed('x', 0.0, ub, lb)
            
            # 15f
            if includeX:
                new_ub = [("px%d" % i, 0.0) for i in includeX]
                model.variables.set_upper_bounds(new_ub)

            # 15b
            new_senses = [("15cy%d" % i, "E") for i in includeY]
            model.linear_constraints.set_senses(new_senses)


        # model.write("maxModel%s_%s.lp" % (mode, timeCounter.time()))
        model.solve()

        if model.solution.get_status() == 1:
            return model.solution.get_objective_value()        
        elif model.solution.get_status() == 3:
            return -1000.0
        else:
            return -1.0

    def init52(self):
        model = cplex.Cplex()
        model.parameters.threads.set(self._threadsNum)
        disableOutput(model) 

        v_lb = []
        v_ub = []
        v_obj = []
        v_varName = []

        v_constraints = []
        v_rhs = []
        v_names = []
        v_senses = []

        varDict = {}
        constDict = {}

        count = 0
        ## Variables
        # p
        for i in range(self._instance._xNum):
            varDict[i] = {}
            for j in range(self._instance._yNum):
                varDict[i][j] = count
                count += 1
                v_lb.append(0.0)
                v_ub.append(1.0)
                v_obj.append(0.0)
                v_varName.append("px%dy%d" % (i, j))

        v_lb.append(0.0)
        v_ub.append(cplex.infinity)
        v_obj.append(1.0)
        v_varName.append("vx")

        v_lb.append(0.0)
        v_ub.append(cplex.infinity)
        v_obj.append(1.0)
        v_varName.append("vy")

        # initially, variables are all not fixed to 0
        cstat = [model.start.status.at_lower_bound for i in range(self._instance._xNum * self._instance._yNum)] + [model.start.status.basic for i in range(2)]


        count = 0
        ## Constraints
        # 18c: x (xNum * xNum)
        for i in range(self._instance._xNum):
            p_idx = [varDict[i][j] for j in range(self._instance._yNum)]
            for i_hat in range(self._instance._xNum):
                values = [(self._instance._value[i][j]['x'] - self._instance._value[i_hat][j]['x']) for j in range(self._instance._yNum)]    
                v_constraints.append(cplex.SparsePair(ind=p_idx, val=values))
                v_rhs.append(0.0)
                v_names.append("18cx%d_%d" %(i, i_hat))
                v_senses.append("G")
                constDict["18cx%d_%d" %(i, i_hat)] = count
                count += 1

        # 18c: y (yNum * yNum)
        for j in range(self._instance._yNum):
            p_idx = [varDict[i][j] for i in range(self._instance._xNum)]
            for j_hat in range(self._instance._yNum):
                values = [(self._instance._value[i][j]['y'] - self._instance._value[i][j_hat]['y']) for i in range(self._instance._xNum)]
                v_constraints.append(cplex.SparsePair(ind=p_idx, val=values))
                v_rhs.append(0.0)
                v_names.append("18cy%d_%d" %(j, j_hat))
                v_senses.append("G")
                constDict["18cy%d_%d" %(j, j_hat)] = count
                count += 1

        # 18d: x (1)
        vx_idx = len(v_lb) - 2
        idx = []
        values = []
        for i in range(self._instance._xNum):
            for j in range(self._instance._yNum):
                idx.append(varDict[i][j])
                values.append(self._instance._value[i][j]['x'])
        v_constraints.append(cplex.SparsePair(ind=idx + [vx_idx], val=values + [-1.0]))
        v_rhs.append(0.0)
        v_names.append("18dx")
        v_senses.append("E")
        constDict["18dx"] = count
        count += 1



        # 18d: y (1)
        vy_idx = len(v_lb) - 1
        idx = []
        values = []
        for i in range(self._instance._xNum):
            for j in range(self._instance._yNum):
                idx.append(varDict[i][j])
                values.append(self._instance._value[i][j]['y'])
        v_constraints.append(cplex.SparsePair(ind=idx + [vy_idx], val=values + [-1.0]))
        v_rhs.append(0.0)
        v_names.append("18dy")
        v_senses.append("E")
        constDict["18dy"] = count
        count += 1

        # 18f: x (xNum)
        for i in range(self._instance._xNum):
            idx = []
            values = []
            for ii in range(self._instance._xNum):
                for j in range(self._instance._yNum):
                    idx.append(varDict[ii][j])
                    values.append(self._instance._value[i][j]['x'])
            v_constraints.append(cplex.SparsePair(ind=idx + [vx_idx], val=values + [-1.0]))
            v_rhs.append(0.0)
            v_names.append("18fx%d" % i)
            v_senses.append("L")
            constDict["18fx%d" % i] = count
            count += 1

        # 18f: y (yNum)
        for j in range(self._instance._yNum):
            idx = []
            values = []
            for jj in range(self._instance._yNum):
                for i in range(self._instance._xNum):
                    idx.append(varDict[i][jj])
                    values.append(self._instance._value[i][j]['y'])
            v_constraints.append(cplex.SparsePair(ind=idx + [vy_idx], val=values + [-1.0]))
            v_rhs.append(0.0)
            v_names.append("18fy%d" % j)
            v_senses.append("L")
            constDict["18fy%d" % j] = count
            count += 1

        # 18g (1)
        idx = [i for i in range(len(v_ub)-2)]
        values = [1.0 for i in range(len(v_ub)-2)]
        v_constraints.append(cplex.SparsePair(ind=idx, val=values))
        v_rhs.append(1.0)
        v_names.append("P")
        v_senses.append("E")
        constDict["P"] = count

        # 18c (xNum**2, yNum**2)G, 18d (1 + 1)E, 18f (xNum + yNum)L, 18g (1)E
        rstat = [model.start.status.basic for i in range(self._instance._xNum ** 2 + self._instance._yNum ** 2)] + [model.start.status.at_lower_bound for i in range(2)] + [model.start.status.basic for i in range(self._instance._xNum + self._instance._yNum)] + [model.start.status.at_lower_bound]

        model.variables.add(names=v_varName, lb=v_lb, ub=v_ub, obj=v_obj)
        model.objective.set_sense(model.objective.sense.maximize)
        model.linear_constraints.add(lin_expr=v_constraints, rhs=v_rhs, names=v_names, senses=v_senses)

        return model, (cstat, rstat), (varDict, constDict)

    def getV_52(self, ub, lb):
        model = cplex.Cplex(self._model52)
        disableOutput(model)
        cstat, rstat = self._start
        varDict, constDict = self._dicts

        x_fix0, x_Nfix0 = self.get_fixed("x", 0.0, ub, lb)
        y_fix0, y_Nfix0 = self.get_fixed("y", 0.0, ub, lb)
        x_fix1, x_Nfix1 = self.get_fixed("x", 1.0, ub, lb)
        y_fix1, y_Nfix1 = self.get_fixed("y", 1.0, ub, lb)

        # variables
        if x_fix0 or y_fix0:
            new_ub = []
            for i in x_fix0:
                for j in range(self._instance._yNum):
                    #cstat[varDict[i][j]] = model.start.status.at_lower_bound
                    new_ub.append(("px%dy%d" % (i, j), 0.0))
            for i in range(self._instance._xNum):
                for j in y_fix0:
                    #cstat[varDict[i][j]] = model.start.status.at_lower_bound
                    new_ub.append(("px%dy%d" % (i, j), 0.0))
            
            model.variables.set_upper_bounds(new_ub)

        
        # constraints
        new_sense = []
        removed = []

        # 18b, fix 18c
        if x_fix1 or y_fix1:
            for i in x_fix1:
                for ii in range(self._instance._xNum):
                    new_sense.append(("18cx%d_%d" % (ii, i), "E"))
                    rstat[constDict["18cx%d_%d" % (ii, i)]] = model.start.status.basic
            for j in y_fix1:
                for jj in range(self._instance._yNum):
                    new_sense.append(("18cy%d_%d" % (jj, j), "E"))
                    rstat[constDict["18cy%d_%d" % (jj, j)]] = model.start.status.basic

        # 18e, fix 18f
            for i in x_fix1:
                new_sense.append(("18fx%d" % i, "E"))
                rstat[constDict["18fx%d" % i]] = model.start.status.basic
            for j in y_fix1:
                new_sense.append(("18fy%d" % j, "E"))
                rstat[constDict["18fy%d" % j]] = model.start.status.basic

            model.linear_constraints.set_senses(new_sense)

        # # update 18b
        # if x_fix0 or y_fix0:
        #     for i in x_fix0:
        #         for ii in range(self._instance._xNum):
        #             removed.append("18cx%d_%d" % (i, ii))
        #     for j in y_fix0:
        #         for jj in range(self._instance._yNum):
        #             removed.append("18cy%d_%d" % (j, jj))

        #     model.linear_constraints.delete(removed)
        #     removed_idx = [constDict[name] for name in removed]
        #     rstat = [rstat[i] for i in range(len(rstat)) if i not in removed_idx]

        model.start.set_start(col_status=cstat, row_status=rstat, col_primal=[], row_primal=[], col_dual=[], row_dual=[])

        model.solve()

        if model.solution.get_status() == 1 or model.solution.get_status() == 5:
            return True, model.solution.get_objective_value(), model.solution.get_values()[:-2]

        elif model.solution.get_status() == 3:
            return True, -100000000000000, None
        else:
            return False, None, None