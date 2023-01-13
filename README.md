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

### Instance Mapping
`Name` and `# of action` corresponds to `Name` and `# of action` in computational results from Table 2 & 3 of the paper submitted to INFORMS Journal of Computing. `Instance Name` corresponds to the name of instances in the Instance folder.

| Name | Instance Name | # of action |
|------|---------------|-------------|
| a1   | ins4          | 100         |
| a2   | ins3          | 100         |
| a3   | ins2          | 100         |
| a4   | ins5          | 100         |
| a5   | ins8          | 100         |
| a6   | ins6          | 100         |
| a7   | ins1          | 100         |
| a8   | ins0          | 100         |
| a9   | ins7          | 100         |
| a10  | ins9          | 100         |
| a11  | ins3          | 105         |
| a12  | ins4          | 105         |
| a13  | ins5          | 105         |
| a14  | ins2          | 105         |
| a15  | ins8          | 105         |
| a16  | ins1          | 105         |
| a17  | ins6          | 105         |
| a18  | ins7          | 105         |
| a19  | ins0          | 105         |
| a20  | ins9          | 105         |
| a21  | ins9          | 110         |
| a22  | ins0          | 110         |
| a23  | ins7          | 110         |
| a24  | ins6          | 110         |
| a25  | ins1          | 110         |
| a26  | ins8          | 110         |
| a27  | ins2          | 110         |
| a28  | ins5          | 110         |
| a29  | ins4          | 110         |
| a30  | ins3          | 110         |
| b1   | ins1          | 120         |
| b2   | ins6          | 120         |
| b3   | ins8          | 120         |
| b4   | ins9          | 120         |
| b5   | ins7          | 120         |
| b6   | ins0          | 120         |
| b7   | ins3          | 120         |
| b8   | ins4          | 120         |
| b9   | ins5          | 120         |
| b10  | ins2          | 120         |
| b11  | ins4          | 125         |
| b12  | ins3          | 125         |
| b13  | ins2          | 125         |
| b14  | ins5          | 125         |
| b15  | ins6          | 125         |
| b16  | ins1          | 125         |
| b17  | ins8          | 125         |
| b18  | ins9          | 125         |
| b19  | ins0          | 125         |
| b20  | ins7          | 125         |
| b21  | ins4          | 130         |
| b22  | ins3          | 130         |
| b23  | ins2          | 130         |
| b24  | ins5          | 130         |
| b25  | ins8          | 130         |
| b26  | ins6          | 130         |
| b27  | ins1          | 130         |
| b28  | ins0          | 130         |
| b29  | ins7          | 130         |
| b30  | ins9          | 130         |
