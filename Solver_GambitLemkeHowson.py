import cplex
from cplex.exceptions import CplexError
import pickle
import csv
import numpy as np
import gambit
from gambit.nash import ExternalSolver
import time as timeCounter
import math

from util import *
from func_timeout import func_timeout, func_set_timeout
import os
import sys
import argparse

class ExternalEnumMixedSolver2(ExternalSolver):
    def solve(self, game, rational=False):
        if not game.is_perfect_recall:
            raise RuntimeError("Computing equilibria of games with imperfect recall is not supported")
        if rational:
            command_line = "gambit-enummixed -L"
        else:
            command_line = "gambit-enummixed -d 10 -L"
        return self._parse_output(self.launch(command_line, game), game, rational)


@func_set_timeout(86400)
def lemkeSolve(g):
    print("start lh")
    try:
        start_time = timeCounter.time()
        result = gambit.nash.lcp_solve(g, use_strategic=True)
        lemke_time = timeCounter.time() - start_time
    except Exception as e:
        print(e)
        lemke_time = 1.0
        result = str(e)
    print("finish lh")
    return lemke_time, result


def script(instance, lemkeOut, timelimit, threadNum): # add sum of support for X equals Y --> may need to deactivate
    X = np.array(instance._X.astype(int), dtype=gambit.Rational)
    Y = np.array(instance._Y.astype(int), dtype=gambit.Rational)

    g = gambit.Game.from_arrays(X, Y)
    lemke_time, result = lemkeSolve(g)
    with open(lemkeOut, "a") as fout:
        csvWriter = csv.writer(fout)
        csvWriter.writerow([instance._xNum, instance._yNum, lemke_time, str(result)[:100]])
        fout.flush()

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
    parser.add_argument("--lemkeOut", type=str, default="lemkeOut6.csv", help="result file for master problem")
    parser.add_argument("--timelimit", type=float, default=100.0, help="time limit")
    parser.add_argument("--threadsNum", type=int, default=1, help="threads number parameter")
    parser.add_argument("--insNum", type=str, default=None, help="instance number")


    args = parser.parse_args()
    lemkeOut = "result/%s" % args.lemkeOut

    with open(lemkeOut, "w") as fout:
        csvWriter = csv.writer(fout)
        csvWriter.writerow(["size1", "size2", "timeL", "output"])


    x = (args.xAction, (args.xMin, args.xMax))
    y = (args.yAction, (args.yMin, args.yMax))
    if args.create:
        createInstance(x,y,args.total)

    instances = loadInstance(x,y)

    if args.insNum == None:
        for i in range(len(instances)):
            print("********** %d ***********\n" % i)
            script(instances[i], lemkeOut, args.timelimit, args.threadsNum)
    else:
        insNums = args.insNum.split(",")
        for i in range(len(instances)):
            if str(i) in insNums:
                print("********** %d ***********\n" % int(i))
                script(instances[int(i)], lemkeOut, args.timelimit, args.threadsNum)

if __name__ == "__main__":
    
    wrapper()
    


    