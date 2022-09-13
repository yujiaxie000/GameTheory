# Enhance with warmstart and heuristic
import cplex
import csv
import numpy as np
import time as timeCounter
import math

from cplex.callbacks import HeuristicCallback, LazyConstraintCallback, UserCutCallback, IncumbentCallback
from init_util import *
from util import *
from addPrimal import *
import sys
from getMax import GetMax
from separationEnhance import *
import argparse

add_heuristic = False
add_corresponding = False
correspondingAction = []
epsilon = 0.0001
lastNodeID = np.Inf
data = []
usercutNum = 0

class MyHeuristicCallBack(HeuristicCallback):
    def __call__(self):
        global epsilon, add_heuristic, data, add_corresponding, correspondingAction
        if add_heuristic:
            self.set_solution(cplex.SparsePair(ind=[i for i in range(self._instance._xNum + self._instance._yNum + 2)], val=data),
                              objective_value=data[-1]+data[-2])
            add_heuristic = False

        if add_corresponding:
            self.set_solution(cplex.SparsePair(ind=[i for i in range(self._instance._xNum + self._instance._yNum + 2)],
                                                val=correspondingAction), objective_value=correspondingAction[-1]+correspondingAction[-2])
            correspondingAction = []
            add_corresponding = False

class MyIncumbentCallBack(IncumbentCallback):
    def __call__(self):
        supports = self._instance.convert(self.get_values())

        resultX, objX = solve_addPrimalX(supports, self._instance)
        resultY, objY = solve_addPrimalY(supports, self._instance)

        if not resultX or not resultY:
            self.reject()
            

class MyLazyConstraintCallBack(LazyConstraintCallback):
    def __call__(self):
        mySeparation = self._separation
        supports = self._instance.convert(self.get_values())

        try:
            
            feas_52=True
                
            global lastNodeID, usercutNum
            if lastNodeID != self.get_node_ID():
                lastNodeID = self.get_node_ID()
                feas_52 = self.add_cut_52()

                   
            if feas_52== True:
                
                resultX, resultY = mySeparation.separate(supports, self._instance)

                if not resultX and resultY:
                    self.add_local(constraint=mySeparation._lazy_constraint[0], sense="G", rhs=mySeparation._lazy_rhs[0])
                    usercutNum += 1
                if not resultY and resultX:
                    self.add_local(constraint=mySeparation._lazy_constraint[1], sense="G", rhs=mySeparation._lazy_rhs[1])
                    usercutNum += 1
                if not resultX and not resultY:
                    self.add_local(constraint=mySeparation._lazy_constraint[0], sense="G", rhs=mySeparation._lazy_rhs[0])
                    self.add_local(constraint=mySeparation._lazy_constraint[1], sense="G", rhs=mySeparation._lazy_rhs[1])
                    usercutNum += 2
                    
                if resultX and resultY:
                    global add_heuristic, data
                    add_heuristic = True
                    data = self.get_values()[:-2]
    
                    resultX, objX = solve_addPrimalX(supports, self._instance)
                    resultY, objY = solve_addPrimalY(supports, self._instance)
                    self._feas51X, self._feas51Y = self._getMax5.getV_51(self.get_upper_bounds(), self.get_lower_bounds())
                    
                    data=np.append(data, [objX, objY])
                    self.add_optimality_cut(objX, objY)


   
        except:
          print(sys.exc_info()[0])
          raise

    def add_optimality_cut(self, objX, objY):
        global epsilon, usercutNum
        data = self.get_values()

        zx = data[-2]
        zy = data[-1]

        zx_idx = len(data) - 2
        zy_idx = zx_idx + 1

        xSupport = np.array(data[:self._instance._xNum])
        ySupport = np.array(data[self._instance._xNum: self._instance._xNum + self._instance._yNum])
        xTemp = np.absolute(xSupport - 1)
        yTemp = np.absolute(ySupport - 1)
        supportX = np.where(xTemp <= epsilon)[0]
        supportX_M = np.where(xTemp > epsilon)[0]
        supportY = np.where(yTemp <= epsilon)[0]
        supportY_M = np.where(yTemp > epsilon)[0]

        if zx > objX + epsilon:
            coef = self._feas51X - objX
            newRhs = objX + len(supportX) * coef 
            indices = np.append(supportX, self._instance.getIndicesY(supportY_M))
            indices = np.append(indices, [zx_idx])
            values = np.append([coef for i in supportX], [-coef for j in supportY_M])
            values = np.append(values, [1])

            newConstraint = cplex.SparsePair(ind=indices.tolist(),val=values)
            self.add_local(constraint=newConstraint, sense="L", rhs=newRhs)
            usercutNum += 1
        
        if zy > objY + epsilon:
            coef = self._feas51Y - objY
            newRhs = objY + len(supportY) * coef
            indices = np.append(supportX_M, self._instance.getIndicesY(supportY))
            indices = np.append(indices, [zy_idx])
            values = np.append([-coef for i in supportX_M], [coef for j in supportY])
            values = np.append(values, [1])
            newConstraint = cplex.SparsePair(ind=indices.tolist(),val=values)
            self.add_local(constraint=newConstraint, sense="L", rhs=newRhs)    
            usercutNum += 1   

    def add_cut_51(self):
        data = self.get_values()
        ub = self.get_upper_bounds()
        lb = self.get_lower_bounds()
        v1, v2 = self._getMax5.getV_51(ub, lb)
        global usercutNum
        try:
            if not (v1 is None):
                if v1 >= 0.0:
                    zx_idx = len(data) - 2
                    zx_constraint = cplex.SparsePair(ind=[zx_idx], val=[1.0])
                    self.add_local(zx_constraint, sense="L", rhs=v1)
                    usercutNum += 1
                elif v1<=-10.0:
                    z_constraint = cplex.SparsePair(ind=[len(ub)-3], val=[1.0])
                    self.add_local(z_constraint, "L", -1.0)
                    usercutNum += 1
            if not (v2 is None):
                if v2 >=0.0:
                    zy_idx = len(data) - 1
                    zy_constraint = cplex.SparsePair(ind=[zy_idx], val=[1.0])
                    self.add_local(zy_constraint, sense="L", rhs=v2)
                    usercutNum += 1
                elif v2<=-10.0:
                    z_constraint = cplex.SparsePair(ind=[len(ub)-3], val=[1.0])
                    self.add_local(z_constraint, "L", -1.0)
                    usercutNum += 1
        except:
          print(sys.exc_info()[0])
          raise

    def preprocess52(self, ub):
        ub = np.array(ub[:-2])
        x_ub = ub[:self._instance._xNum]
        y_ub = ub[self._instance._xNum:]

        x_remain = np.argwhere(np.absolute(x_ub - 0) > 0.000001).flatten()
        y_remain = np.argwhere(np.absolute(y_ub - 0) > 0.000001).flatten()

        utility = self._instance._X + self._instance._Y

        utility = np.take(utility, x_remain, axis=0)
        utility = np.take(utility, y_remain, axis=1)
        return np.max(utility)

    def add_cut_52(self):
        data = self.get_values()
        ub = self.get_upper_bounds()
        lb = self.get_lower_bounds()

        feas =True
        
        pre_val= self.preprocess52(ub)
        global usercutNum
        if  pre_val < self.get_incumbent_objective_value():
            v = pre_val
            
            zx_idx = len(data) - 2
            zy_idx = zx_idx + 1
            z_constraint = cplex.SparsePair(ind=[zx_idx, zy_idx], val=[1.0,  1.0])
            self.add_local(constraint=z_constraint, sense="L", rhs=v)
            feas=False
            usercutNum += 1

        else:

            v1, v2 = self._getMax5.getV_51(ub, lb)
            if  v1 >= 0.0:
                zx_idx = len(data) - 2
                zx_constraint = cplex.SparsePair(ind=[zx_idx], val=[1.0])
                self.add_local(zx_constraint, sense="L", rhs=v1)
                usercutNum += 1
            if v2 >=0.0:
                zy_idx = len(data) - 1
                zy_constraint = cplex.SparsePair(ind=[zy_idx], val=[1.0])
                self.add_local(zy_constraint, sense="L", rhs=v2)    
                usercutNum += 1
            if v1 < -10 or v2 < -10:
                z_constraint = cplex.SparsePair(ind=[len(ub)-3], val=[1.0])
                self.add_local(constraint=z_constraint, sense="L", rhs=-1.0)
                feas=False
                usercutNum += 1

            else:

                status, v, solution = self._getMax5.getV_52(ub, lb)
            
                if status and v >= 0:

                    zx_idx = len(data) - 2
                    zy_idx = zx_idx + 1

                    z_constraint = cplex.SparsePair(ind=[zx_idx, zy_idx], val=[1.0,  1.0])
                    self.add_local(constraint=z_constraint, sense="L", rhs=v)
                    usercutNum += 1

                    if self.get_incumbent_objective_value()+epsilon<= v:
                        global add_corresponding, correspondingAction
                        correspondingAction= self._instance.getCorrespondingAction(solution)
                        correspondingAction= np.append(correspondingAction, [0.0, 0.0])
                        
                        sum_x=0                        
                        for i in range(self._instance._xNum):
                            sum_x+=correspondingAction[i]
                        sum_y=0
                        for i in range(self._instance._yNum):
                            sum_y+=correspondingAction[self._instance._xNum+i]
                        if sum_x==sum_y:
                            supports = self._instance.convert(correspondingAction)
                            supports = self._instance.convert(correspondingAction)
                            resultX, objX = solve_addPrimalX(supports, self._instance)
                            if resultX:
                                resultY, objY = solve_addPrimalY(supports, self._instance)
                                if resultY:
                                    if objX + objY > self.get_incumbent_objective_value():
                                        correspondingAction[-2] = objX
                                        correspondingAction[-1] = objY
                                        add_corresponding = True

                elif status and v < 0:
                    z_constraint = cplex.SparsePair(ind=[len(ub)-3], val=[1.0])
                    self.add_local(constraint=z_constraint, sense="L", rhs=-1.0)
                    feas=False
                    usercutNum += 1
                    
                
        return feas



class MyUserCutCallBack(UserCutCallback):
    def __call__(self):
        mySeparation = self._separation
        supports = self._instance.convert(self.get_values())

        
        try:
                               
            global lastNodeID
 
            if lastNodeID != self.get_node_ID():
                lastNodeID = self.get_node_ID()
                feas_52=self.add_cut_52()
                


        except:
          print(sys.exc_info()[0])
          raise

  
    def add_cut_51(self):
        data = self.get_values()
        ub = self.get_upper_bounds()
        lb = self.get_lower_bounds()
        v1, v2 = self._getMax5.getV_51(ub, lb)

        global usercutNum
        try:
            if not (v1 is None):
                if v1 >= 0.0:
                    zx_idx = len(data) - 2
                    zx_constraint = cplex.SparsePair(ind=[zx_idx], val=[1.0])
                    self.add_local(zx_constraint, sense="L", rhs=v1)
                    usercutNum += 1
                elif v1<=-10.0:
                    z_constraint = cplex.SparsePair(ind=[len(ub)-3], val=[1.0])
                    self.add_local(z_constraint, "L", -1.0)
                    usercutNum += 1
            if not (v2 is None):
                if v2 >=0.0:
                    zy_idx = len(data) - 1
                    zy_constraint = cplex.SparsePair(ind=[zy_idx], val=[1.0])
                    self.add_local(zy_constraint, sense="L", rhs=v2)
                    usercutNum += 1
                elif v2<=-10.0:
                    z_constraint = cplex.SparsePair(ind=[len(ub)-3], val=[1.0])
                    self.add_local(z_constraint, "L", -1.0)
                    usercutNum += 1
        except:
          print(sys.exc_info()[0])
          raise

    def preprocess52(self, ub):
        ub = np.array(ub[:-2])
        x_ub = ub[:self._instance._xNum]
        y_ub = ub[self._instance._xNum:]

        x_remain = np.argwhere(np.absolute(x_ub - 0) > 0.000001).flatten()
        y_remain = np.argwhere(np.absolute(y_ub - 0) > 0.000001).flatten()

        utility = self._instance._X + self._instance._Y

        utility = np.take(utility, x_remain, axis=0)
        utility = np.take(utility, y_remain, axis=1)
        return np.max(utility)



    def add_cut_52(self):
        data = self.get_values()
        ub = self.get_upper_bounds()
        lb = self.get_lower_bounds()

        feas =True
        
        pre_val= self.preprocess52(ub)
        global usercutNum
        if  pre_val < self.get_incumbent_objective_value():
            v = pre_val
            
            zx_idx = len(data) - 2
            zy_idx = zx_idx + 1
            z_constraint = cplex.SparsePair(ind=[zx_idx, zy_idx], val=[1.0,  1.0])
            self.add_local(cut=z_constraint, sense="L", rhs=v)
            feas=False
            usercutNum += 1
        
        else:

            v1, v2 = self._getMax5.getV_51(ub, lb)
            if  v1 >= 0.0:
                zx_idx = len(data) - 2
                zx_constraint = cplex.SparsePair(ind=[zx_idx], val=[1.0])
                self.add_local(cut=zx_constraint, sense="L", rhs=v1)
                usercutNum += 1
            if v2 >=0.0:
                zy_idx = len(data) - 1
                zy_constraint = cplex.SparsePair(ind=[zy_idx], val=[1.0])
                self.add_local(cut=zy_constraint, sense="L", rhs=v2)  
                usercutNum += 1  
                
            if v1 < -10 or v2 < -10:
                z_constraint = cplex.SparsePair(ind=[len(ub)-3], val=[1.0])
                self.add_local(cut=z_constraint, sense="L", rhs=-1.0)
                feas=False
                usercutNum += 1

            else:

                status, v, solution = self._getMax5.getV_52(ub, lb)

            
                if status and v >= 0:

                    zx_idx = len(data) - 2
                    zy_idx = zx_idx + 1

                    z_constraint = cplex.SparsePair(ind=[zx_idx, zy_idx], val=[1.0,  1.0])
                    self.add_local(cut=z_constraint, sense="L", rhs=v)
                    usercutNum += 1

                    if  self.get_incumbent_objective_value()+epsilon<= v:
                        global add_corresponding, correspondingAction
                        # mySeparation = self._separation
                        correspondingAction= self._instance.getCorrespondingAction(solution)
                        correspondingAction= np.append(correspondingAction, [0.0, 0.0])
                        sum_x=0                        
                        for i in range(self._instance._xNum):
                            sum_x+=correspondingAction[i]
                        sum_y=0
                        for i in range(self._instance._yNum):
                            sum_y+=correspondingAction[self._instance._xNum+i]
                        if sum_x==sum_y:
                            supports = self._instance.convert(correspondingAction)
                            resultX, objX = solve_addPrimalX(supports, self._instance)
                            if resultX:
                                resultY, objY = solve_addPrimalY(supports, self._instance)
                                if resultY:
                                    if objX + objY > self.get_incumbent_objective_value():
                                        correspondingAction[-2] = objX
                                        correspondingAction[-1] = objY
                                        add_corresponding = True

                elif status and v < 0:
                    z_constraint = cplex.SparsePair(ind=[len(ub)-3], val=[1.0])
                    self.add_local(cut=z_constraint, sense="L", rhs=-1.0)
                    feas=False
                    usercutNum += 1
                    
                
        return feas

class MyLazyConstraintBendersCallBack(LazyConstraintCallback):
    def __call__(self):
        mySeparation = self._separation
        supports = self._instance.convert(self.get_values())
        zx, zy = self.get_values()[-2:]
        
        try:
            statusX, statusY = mySeparation.separate(supports, self._instance, zx, zy)

            if mySeparation._lazy_constraint[0] != 0:
                self.add_local(constraint=mySeparation._lazy_constraint[0], sense="G", rhs=mySeparation._lazy_rhs[0])
            if mySeparation._lazy_constraint[1] != 0:
                self.add_local(constraint=mySeparation._lazy_constraint[1], sense="G", rhs=mySeparation._lazy_rhs[1])

        except:
            print(sys.exc_info()[0])
            raise


def script(instance, masterOut, timelimit, threadNum):
    global usercutNum
    usercutNum = 0

    masterPEnhance = cplex.Cplex()
    masterPEnhance, preprocessTime, processType, warmStartSolution = initMaster(masterPEnhance, instance)

    masterPEnhance.parameters.preprocessing.presolve.set(masterPEnhance.parameters.preprocessing.presolve.values.off)
    masterPEnhance.parameters.mip.strategy.heuristicfreq.set(-1)
    masterPEnhance.parameters.threads.set(threadNum)
    masterPEnhance.parameters.timelimit.set(timelimit)
    masterPEnhance.parameters.workmem.set(16000)
    separationPEnhance = MySeparationEnhance(instance, threadNum)
    getMax5Enhance = GetMax(instance, threadNum)
    
    separationPEnhanceUser = MySeparationEnhance(instance, threadNum)
    getMax5EnhanceUser = GetMax(instance, threadNum)
    if processType:
        branchCallbackEnhance = masterPEnhance.register_callback(MyBranchCallBackEnhance)
    else:
        branchCallbackEnhance = masterPEnhance.register_callback(MyBranchCallBackEnhance2)
    branchCallbackEnhance._separation = separationPEnhance
    branchCallbackEnhance._instance = instance
    lazyConstraintEnhance = masterPEnhance.register_callback(MyLazyConstraintCallBack)
    lazyConstraintEnhance._separation = separationPEnhance
    lazyConstraintEnhance._instance = instance
    lazyConstraintEnhance._getMax5 = getMax5Enhance


    heuristicConstraintEnhance = masterPEnhance.register_callback(MyHeuristicCallBack)
    heuristicConstraintEnhance._instance = instance

    incumbentEnhance = masterPEnhance.register_callback(MyIncumbentCallBack)
    incumbentEnhance._instance = instance

    userCutEnhance = masterPEnhance.register_callback(MyUserCutCallBack)
    userCutEnhance._separation = separationPEnhanceUser
    userCutEnhance._instance = instance
    userCutEnhance._getMax5 = getMax5EnhanceUser


    masterPEnhance.parameters.mip.strategy.search.set(masterPEnhance.parameters.mip.strategy.search.values.traditional)
    masterPEnhance.parameters.mip.strategy.variableselect.set(3)
 
    masterPEnhance.parameters.workmem.set(20480)
    start_time = timeCounter.time()
    print("+++++++++++++++++++++ Enhance")
    masterPEnhance.solve()
    masterPEnhance_time = timeCounter.time() - start_time

    masterPEnhance_status = masterPEnhance.solution.get_status()
    
    if masterPEnhance_status == 101 or masterPEnhance_status == 102 or masterPEnhance_status == 107:
        optval = masterPEnhance.solution.get_values()
        supportsize=0
        for i in range(instance._xNum+instance._yNum):
            supportsize +=optval[i]
        with open(masterOut, "a") as fout:
            csvWriter = csv.writer(fout)
            csvWriter.writerow([instance._xNum, instance._yNum, masterPEnhance_status, masterPEnhance.solution.get_objective_value(), masterPEnhance_time, masterPEnhance.solution.progress.get_num_nodes_processed(), masterPEnhance.solution.MIP.get_mip_relative_gap(), supportsize, usercutNum, preprocessTime])
    else:
        with open(masterOut, "a") as fout:
            csvWriter = csv.writer(fout)
            csvWriter.writerow([instance._xNum, instance._yNum, masterPEnhance_status, -1, masterPEnhance_time, masterPEnhance.solution.progress.get_num_nodes_processed(), -1, usercutNum, preprocessTime])

def wrapper():
    parser = argparse.ArgumentParser()
    parser.add_argument("--xAction", type=int, default=6, help="number of actions for row player")
    parser.add_argument("--yAction", type=int, default=6, help="number of actions for column player")
    parser.add_argument("--xMin", type=float, default=100.0, help="min utility for row player")
    parser.add_argument("--yMin", type=float, default=100.0, help="min utility for column player")
    parser.add_argument("--xMax", type=float, default=200.0, help="max utility for row player")
    parser.add_argument("--yMax", type=float, default=200.0, help="max utility for row player")
    parser.add_argument("--create", type=int, default=0, help="if create new instances")
    parser.add_argument("--total", type=int, default=1, help="total new instance creating")
    parser.add_argument("--masterResult", type=str, default="resultMaster6.csv", help="result file for master problem")
    parser.add_argument("--timelimit", type=float, default=100.0, help="time limit")
    parser.add_argument("--threadsNum", type=int, default=1, help="threads number parameter")


    args = parser.parse_args()
    masterOut = "result/%s" % args.masterResult

    with open(masterOut, "w") as fout:
        csvWriter = csv.writer(fout)
        csvWriter.writerow(["size1", "size2", "statusI", "objValI", "timeI", "numNodesI", "relGapI", "supportSizeMIP", "userCutNum", "preprocessTime"])

    x = (args.xAction, (args.xMin, args.xMax))
    y = (args.yAction, (args.yMin, args.yMax))
    if args.create:
        createInstance(x,y,args.total)
    instances = loadInstance(x,y)

    for i in range(len(instances)):
        print("********** %d ***********\n" % i)
        try:
            script(instances[i], masterOut, args.timelimit, args.threadsNum)
        except:
            print("Failed")


if __name__ == "__main__":
    
    wrapper()
    


    
