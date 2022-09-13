import cplex
import pickle
import csv
import numpy as np
import time as timeCounter

from util import *
from init_util import *
import sys
from MIP import *
import argparse


def script(instance, MIPOut, timelimit, threadNum):
    masterPEnhance = cplex.Cplex()
    masterPEnhance, preprocessTime, processType, warmStartSolution = initMaster(masterPEnhance, instance) 
    
    start_time = timeCounter.time()
    MIPOpt = getOpt(instance, warmStartSolution)    
    MIPOpt.parameters.timelimit.set(timelimit)
    MIPOpt.parameters.workmem.set(20480)
    MIPOpt.parameters.threads.set(threadNum)
    print("+++++++++++++++++++++ MIP")
    MIPOpt.solve()
    MIPOpt_time = timeCounter.time() - start_time
    MIPOpt_status = MIPOpt.solution.get_status()

    if MIPOpt_status == 101 or MIPOpt_status == 102 or MIPOpt_status == 107:
        optval = MIPOpt.solution.get_values()
        MIP_supportsize=0
        for i in range(2*(instance._xNum+instance._yNum), 3*(instance._xNum+instance._yNum)):
            MIP_supportsize +=optval[i]

        with open(MIPOut, "a") as fout:
            csvWriter = csv.writer(fout)
            csvWriter.writerow([instance._xNum, instance._yNum, MIPOpt_status, MIPOpt.solution.get_objective_value(), MIPOpt_time, MIPOpt.solution.progress.get_num_nodes_processed(), MIPOpt.solution.MIP.get_mip_relative_gap(), np.sum([MIPOpt.solution.MIP.get_num_cuts(cutType) for cutType in [0,1,2,3,4,5,6,7,8,9,10,14,15,16,17,18,19,20,21]]), MIP_supportsize])
            
    else:
        with open(MIPOut, "a") as fout:
            csvWriter = csv.writer(fout)
            csvWriter.writerow([instance._xNum, instance._yNum, MIPOpt_status, -1, MIPOpt_time, MIPOpt.solution.progress.get_num_nodes_processed(), -1, np.sum([MIPOpt.solution.MIP.get_num_cuts(cutType) for cutType in [0,1,2,3,4,5,6,7,8,9,10,14,15,16,17,18,19,20,21]])])


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
    parser.add_argument("--mipResult", type=str, default="resultMIP6.csv", help="result file for MIP problem")
    parser.add_argument("--timelimit", type=float, default=100.0, help="time limit")
    parser.add_argument("--threadsNum", type=int, default=1, help="threads number parameter")


    args = parser.parse_args()
    MIPOut = "result/%s" % args.mipResult

    with open(MIPOut, "w") as fout:
        csvWriter = csv.writer(fout)
        csvWriter.writerow(["size1", "size2", "statusM", "objValM", "timeM", "numNodesM", "relGapM", "numCutM", "supportSizeMIP"])

    x = (args.xAction, (args.xMin, args.xMax))
    y = (args.yAction, (args.yMin, args.yMax))
    if args.create:
        createInstance(x,y,args.total)
    instances = loadInstance(x,y)

    for i in range(len(instances)):
        print("********** %d ***********\n" % i)
        try:
            script(instances[i], MIPOut, args.timelimit, args.threadsNum)
        except:
            print("Failed")


if __name__ == "__main__":
    
    wrapper()
    


    
