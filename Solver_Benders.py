# only Benders
import cplex
import csv
import numpy as np
import time as timeCounter

from cplex.callbacks import LazyConstraintCallback
from separationBenders import MySeparationBenders
from util import *
from init_util import *
import sys
import argparse

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

def script(instance, bendersOut, timelimit, threadNum):
    
    masterPEnhance = cplex.Cplex()
    masterPEnhance, preprocessTime, processType, warmStartSolution = initMaster(masterPEnhance, instance) 
    
    masterBenders = cplex.Cplex()
    masterBenders = initMaster_Benders(masterBenders, instance, warmStartSolution)
    masterBenders.parameters.preprocessing.presolve.set(masterBenders.parameters.preprocessing.presolve.values.off)
    masterBenders.parameters.mip.strategy.heuristicfreq.set(-1)
    masterBenders.parameters.threads.set(threadNum)
    masterBenders.parameters.timelimit.set(timelimit)
    masterBenders.parameters.workmem.set(16000)
    separationBenders = MySeparationBenders(instance)
    lazyConstraintBenders = masterBenders.register_callback(MyLazyConstraintBendersCallBack)
    lazyConstraintBenders._instance = instance
    lazyConstraintBenders._separation = separationBenders

    masterBenders.parameters.mip.strategy.search.set(masterBenders.parameters.mip.strategy.search.values.traditional)
    masterBenders.parameters.mip.strategy.variableselect.set(3)
 
    masterBenders.parameters.workmem.set(20480)
    start_time = timeCounter.time()
    print("+++++++++++++++++++++ Benders")
    masterBenders.solve()
    masterBenders_time = timeCounter.time() - start_time

    masterBenders_status = masterBenders.solution.get_status()

    if masterBenders_status == 101 or masterBenders_status == 102 or masterBenders_status == 107:
        optval = masterBenders.solution.get_values()
        supportsize=0
        for i in range(instance._xNum+instance._yNum):
            supportsize +=optval[i]
        with open(bendersOut, "a") as fout:
            csvWriter = csv.writer(fout)
            csvWriter.writerow([instance._xNum, 
                instance._yNum, 
                masterBenders_status, 
                masterBenders.solution.get_objective_value(), 
                masterBenders_time, 
                masterBenders.solution.progress.get_num_nodes_processed(), 
                masterBenders.solution.MIP.get_mip_relative_gap(),
                np.sum([masterBenders.solution.MIP.get_num_cuts(cutType) for cutType in [0,1,2,3,4,5,6,7,8,9,10,14,15,16,17,18,19,20,21]]),
                supportsize])
    else:
        with open(bendersOut, "a") as fout:
            csvWriter = csv.writer(fout)
            csvWriter.writerow([instance._xNum,
                instance._yNum,
                masterBenders_status,
                -1,
                masterBenders_time,
                masterBenders.solution.progress.get_num_nodes_processed(), 
                np.sum([masterBenders.solution.MIP.get_num_cuts(cutType) for cutType in [0,1,2,3,4,5,6,7,8,9,10,14,15,16,17,18,19,20,21]]),
                -1])


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
    parser.add_argument("--bendersResult", type=str, default="resultBenders6.csv", help="result file for Benders problem")
    parser.add_argument("--timelimit", type=float, default=100.0, help="time limit")
    parser.add_argument("--threadsNum", type=int, default=1, help="threads number parameter")


    args = parser.parse_args()
    BendersOut = "result/%s" % args.bendersResult

    with open(BendersOut, "w") as fout:
        csvWriter = csv.writer(fout)
        csvWriter.writerow(["size1", "size2", "statusB", "objValB", "timeB", "numNodesB", "relGapB", "numCutB", "supportSizeB"])

    x = (args.xAction, (args.xMin, args.xMax))
    y = (args.yAction, (args.yMin, args.yMax))
    if args.create:
        createInstance(x,y,args.total)
    instances = loadInstance(x,y)

    for i in range(len(instances)):
        print("********** %d ***********\n" % i)
        try:
            script(instances[i], BendersOut, args.timelimit, args.threadsNum)
        except:
            print(sys.exc_info()[0])
            print("Failed")

if __name__ == "__main__":
    
    wrapper()
    


    
