# GameTheory

## Environment Setup
### Directories and Files
```bash
|--	GameTheory
|	|--	Instance
|	|--	result
|	Solver_GambitLemkeHowson.py
|	Solver_Benders.py
|	Solver_MIP.py
|	Solver_EnhancedNoHeuristic.py
|	Solver_EnhancedHeuristic.py
|	MIP.py
|	separationEnhance.py
|	separationBenders.py
|	addPrimal.py
|	getMax.py
|	GTObj.py
|	util.py
|	init_util.py
```

#### File Explanations
1. Initialization
	- `GTObj.py`: randomly initiate 2-player game theory instances
	- `init_util.py`, `getMax.py`: helper functions to preprocess instance, retrieve warm start solutions, and initiate master problems
2. Helper Functions
	- `MIP.py`: helper functions to solve via MIP
	- `separationEnhance.py`, `addPrimal.py`: solving subproblems using our proposed algorithm 
	- `separationBenders.py`: solving Benders subproblems
	- `util.py`: misc helper functions for data preprocessing and postprocessing
3. Solvers
	- `Solver_GambitLemkeHowson.py`: solve game theory instances by enumerating Lemke-Howson algorithms (Gambit)
	- `Solver_Benders.py`: solve by Benders decomposition
	- `Solver_MIP`: solve by MIP
	- `Solver_EnhancedNoHeuristic.py`: solve by our proposed algorithm but without warm start solution and heuristics
	- `Solver_EnhancedHeuristic`: solve by our proposed algorithm using warm start solutions and heuristics

#### Directory Explanations
1. `Instance`: contain generated instances
2. `result`: contain result outputs

### Required Packages
1. Python 3
2. Gambit and Gambit Python Extension: https://gambitproject.readthedocs.io/en/latest/index.html
3. CPLEX
4. numpy, itertools


### Solver Parameters and Expected Outputs
1. Parameters for all solvers
	- xAction: number of actions for player 1
	- yAction: number of actions for player 2
	- xMin: minimum utility for player 1
	- yMin: minimum utility for player 2
	- xMaX: maximum utility for player 1
	- yMaX: maximum utility for player 2
	- create: create new instances or not (1: create; 0: no create)
	- total: total number of new instances creating
	- timelimit: time limit for the solver
	- threadsNum: number of threads for CPLEX solvers
2. Paremeters for `Solver_GambitLemkeHowson`
	- insNum: instance index to run specific instances (separated using comma)
3. Paremeters for `Solver_Benders`:
	- bendersResult: output filename for benders result
4. Paremeters for `Solver_MIP`:
	- mipResult: output filename for MIP result
5. Paremeters for `Solver_EnhancedHeuristic` and `Solver_EnhancedNoHeuristic`:
	- masterResult: output filename for enhanced solver result

#### Example Execution Inputs
```python
	python Solver_GambitLemkeHowson.py --xAction=20 --yAction=20 --xMin=100.0 --xMax=200.0 --yMin=100.0 --yMax=200.0 --create=0 --total=10 -    -timelimit=21600.0 --threadsNum=1 --lemkeOut=lemke20.csv
```

```python
	python Solver_Benders.py --xAction=100 --yAction=100 --xMin=100.0 --xMax=200.0 --yMin=100.0 --yMax=20    0.0 --create=0 --total=10 --threadsNum=1 --timelimit=21600.0 --bendersResult=resultBender100.csv

```
