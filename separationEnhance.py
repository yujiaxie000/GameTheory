import cplex
from cplex.callbacks import LazyConstraintCallback, BranchCallback
from util import *
from addPrimal import *
import numpy as np
import sys
from cplex.exceptions.errors import CplexSolverError

class MyBranchCallBackEnhance(BranchCallback): # get the id to check if it's still in the root node
	def __call__(self):

		if self.get_node_ID() == 0:
			varNum = len(self.get_values()[:-2])

			self.make_branch(1.0, constraints=[(cplex.SparsePair(ind=[i for i in range(varNum)], val=[1.0 for i in range(varNum)]), "E", 4.0)])
			self.make_branch(0.0, constraints=[(cplex.SparsePair(ind=[i for i in range(varNum)], val=[1.0 for i in range(varNum)]), "G", 6.0)])

class MyBranchCallBackEnhance2(BranchCallback): # get the id to check if it's still in the root node
	def __call__(self):

		if self.get_node_ID() == 0:
			varNum = len(self.get_values()[:-2])

			self.make_branch(1.0, constraints=[(cplex.SparsePair(ind=[i for i in range(varNum)], val=[1.0 for i in range(varNum)]), "E", 6.0)])
			self.make_branch(0.0, constraints=[(cplex.SparsePair(ind=[i for i in range(varNum)], val=[1.0 for i in range(varNum)]), "G", 8.0)])


class MySeparationEnhance():
	def __init__(self, instance, threadsNum):
		self._threadNum = threadsNum
		self._indicesX, self._indicesY = self.initIndices(instance)
		self._modelX, self._modelY = self.initSeparationEnhance(instance)
		self._instance = instance
		self._lazy_constraint = [None, None]
		self._lazy_rhs = [None, None]
		disableOutput(self._modelX)
		disableOutput(self._modelY)

	def initIndices(self, instance):
		indicesX = {}
		indicesX['e'] = {}
		for i in range(instance._xNum):
			indicesX[i] = {}
			for j in range(instance._xNum):
				indicesX[i][j] = {}
				indicesX[i][j]['n'] = 0.0

		indicesY = {}
		indicesY['e'] = {}
		for i in range(instance._yNum):
			indicesY[i] = {}
			for j in range(instance._yNum):
				indicesY[i][j] = {}
				indicesY[i][j]['n'] = 0.0

		return indicesX, indicesY

	def initSeparationEnhance(self, instance):
		# for x
		#### Variables
		modelX = cplex.Cplex()
		modelX.parameters.threads.set(self._threadNum)
		self.sx_lb = []
		self.sx_ub = []
		self.sx_objective = []
		self.sx_variableName = []

		# for q-
		for i in range(instance._xNum):
			self.sx_lb.append(-cplex.infinity)
			self.sx_ub.append(0.0)
			self.sx_objective.append(1.0) # should be c, but use 1.0 to test
			self.sx_variableName.append("qxMinus%d" % i)

		# for q+
		for i in range(instance._xNum):
			self.sx_lb.append(0.0)
			self.sx_ub.append(cplex.infinity)
			self.sx_objective.append(0.0)
			self.sx_variableName.append("qxPlus%d" % i)

		self.sx_lb.append(-cplex.infinity)
		self.sx_ub.append(-1)
		self.sx_objective.append(0.0)
		self.sx_variableName.append("Sx")

		modelX.objective.set_sense(modelX.objective.sense.maximize)
		

		#### Constraints
		self.sx_matrix = []
		self.sx_rhs = []
		self.sx_constraint_name = []
		self.sx_constraint_senses = []

		sx_qMinus_indices = [i for i in range(instance._xNum)]
		sx_qPlus_indices = [i + instance._xNum for i in range(instance._xNum)]
		sx_indices = [i for i in range(len(self.sx_lb))]

		for j in range(instance._yNum):
			values = []
			for i in range(len(sx_qMinus_indices)):
				values.append(instance._value[i][j]['x'])
			for i in range(len(sx_qPlus_indices)):
				values.append(instance._value[i][j]['x'])
			values.append(1.0) # for sx

			self.sx_matrix.append(cplex.SparsePair(ind=sx_indices, val=values))
			self.sx_rhs.append(0.0)
			self.sx_constraint_senses.append("G")
			self.sx_constraint_name.append("dy%d" % j)

		self.sx_matrix.append(cplex.SparsePair(ind=sx_indices[:-1], val=[-1.0 for i in range(instance._xNum)] + [-1.0 for i in range(instance._xNum)]))
		self.sx_rhs.append(0.0)
		self.sx_constraint_senses.append("E")
		self.sx_constraint_name.append("Ux")

		self.sx_matrix.append(cplex.SparsePair(ind=[sx_indices[-1]], val=[1.0]))
		self.sx_rhs.append(-1.0)
		self.sx_constraint_senses.append("L")
		self.sx_constraint_name.append("boundxS")
		

		modelX.variables.add(names=self.sx_variableName, lb=self.sx_lb, ub=self.sx_ub, obj=self.sx_objective)
		modelX.linear_constraints.add(rhs=self.sx_rhs, lin_expr=self.sx_matrix, names=self.sx_constraint_name, senses=self.sx_constraint_senses)
		modelX.parameters.preprocessing.reduce.set(0)

		# for y

		#### Variables
		modelY = cplex.Cplex()
		modelY.parameters.threads.set(self._threadNum)
		self.sy_lb = []
		self.sy_ub = []
		# sy_type = ""
		self.sy_objective = []
		self.sy_variableName = []

		# for q-
		for j in range(instance._yNum):
			self.sy_lb.append(-cplex.infinity)
			self.sy_ub.append(0.0)
			self.sy_objective.append(1.0) # should be c, but use 1.0 to test
			self.sy_variableName.append("qyMinus%d" % j)

		# for q+
		for j in range(instance._yNum):
			self.sy_lb.append(0.0)
			self.sy_ub.append(cplex.infinity)
			self.sy_objective.append(0.0)
			self.sy_variableName.append("qyPlus%d" % j)

		self.sy_lb.append(-cplex.infinity)
		self.sy_ub.append(-1)
		self.sy_objective.append(0.0)
		self.sy_variableName.append("Sy")

		modelY.objective.set_sense(modelY.objective.sense.maximize)

		#### Constraints
		self.sy_matrix = []
		self.sy_rhs = []
		self.sy_constraint_name = []
		self.sy_constraint_senses = []

		sy_qMinus_indices = [j for j in range(instance._yNum)]
		sy_qPlus_indices = [j + instance._yNum for j in range(instance._yNum)]
		sy_indices = [j for j in range(len(self.sy_lb))]

		for i in range(instance._xNum):
			values = []
			for j in range(len(sy_qMinus_indices)):
				values.append(instance._value[i][j]['y'])
			for j in range(len(sy_qPlus_indices)):
				values.append(instance._value[i][j]['y'])
			values.append(1.0) # for Sy

			self.sy_matrix.append(cplex.SparsePair(ind=sy_indices, val=values))
			self.sy_rhs.append(0.0)
			self.sy_constraint_senses.append("G")
			self.sy_constraint_name.append("dx%d" % i)

		self.sy_matrix.append(cplex.SparsePair(ind=sy_indices[:-1], val=[-1.0 for i in range(instance._yNum)] + [-1.0 for i in range(instance._yNum)]))
		self.sy_rhs.append(0.0)
		self.sy_constraint_senses.append("E")
		self.sy_constraint_name.append("Uy")

		self.sy_matrix.append(cplex.SparsePair(ind=[sy_indices[-1]], val=[1.0]))
		self.sy_rhs.append(-1.0)
		self.sy_constraint_senses.append("L")
		self.sy_constraint_name.append("boundyS")

		modelY.variables.add(names=self.sy_variableName, lb=self.sy_lb, ub=self.sy_ub, obj=self.sy_objective)
		modelY.linear_constraints.add(rhs=self.sy_rhs, lin_expr=self.sy_matrix, names=self.sy_constraint_name, senses=self.sy_constraint_senses)
		modelY.parameters.preprocessing.reduce.set(0)

		return modelX, modelY

	def separateXEnhance(self, supports, instance):
		modelX = cplex.Cplex(self._modelX)

		disableOutput(modelX)
		supportX, supportY, supportX_M, supportY_M = supports
		for i in supportX_M:
			modelX.variables.set_lower_bounds(i, 0.0)

		
		for j in supportY_M:
			modelX.linear_constraints.delete("dy%d" % j)
		modelX.solve()
		return modelX

	def separateYEnhance(self, supports, instance):
		modelY = cplex.Cplex(self._modelY)
		disableOutput(modelY)
		supportX, supportY, supportX_M, supportY_M = supports
		
		for j in supportY_M:
			modelY.variables.set_lower_bounds(j, 0.0)
		
		for i in supportX_M:
			modelY.linear_constraints.delete("dx%d" % i)
		modelY.solve()
		return modelY

	def separate(self, supports, instance):
		modelX = self.separateXEnhance(supports, instance)
		modelY = self.separateYEnhance(supports, instance)
		modelX_status = modelX.solution.get_status() # 3: infeasible, 1: optimal
		modelY_status = modelY.solution.get_status()

		if modelX_status == 3 and modelY_status == 3:
			return True, True

		if modelX_status == 3 and modelY_status == 1:
			self.constructLazyConstraint(modelX, modelY, supports, instance, 1)
			return True, False

		if modelX_status == 1 and modelY_status == 3:
			self.constructLazyConstraint(modelX, modelY, supports, instance, 0)
			return False, True

		if modelX_status == 1 and modelY_status == 1:
			self.constructLazyConstraint(modelX, modelY, supports, instance, 2)
			return False, False


	def constructLazyConstraint(self, modelX, modelY, supports, instance, mode):
		supportX, supportY, supportX_M, supportY_M = supports
		newSupportX = supportX
		newSupportY = supportY
		newSupportX_M = supportX_M
		newSupportY_M = supportY_M
		
		if mode == 0 or mode == 2: # x
			newSupportX = []
			newSupportX_M = []
			vals = modelX.solution.get_values()
			for i in range(instance._xNum):
				if vals[i] < 0:
					newSupportX.append(i)
				else:
					newSupportX_M.append(i)
			if len(newSupportX) == 0:
				newSupportX = supportX
				newSupportX_M = supportX_M

			rhs = 1.0 - len(newSupportX)
			indices = newSupportX + instance.getIndicesY(supportY_M)
			values = [-1 for i in newSupportX] + [1 for j in supportY_M]
			self._lazy_constraint[0] = cplex.SparsePair(ind=indices,val=values)
			self._lazy_rhs[0] = rhs
		
		if mode == 1 or mode == 2: # for y
			newSupportY = []
			newSupportY_M = []
			vals = modelY.solution.get_values()
			for j in range(instance._yNum):
				if vals[j] < 0:
					newSupportY.append(j)
				else:
					newSupportY_M.append(j)
			if len(newSupportY) == 0:
				newSupportY = supportY
				newSupportY_M = supportY_M

			rhs = 1.0 - len(newSupportY)
			indices = supportX_M + instance.getIndicesY(newSupportY)
			values = [1 for i in supportX_M] + [-1 for j in newSupportY]
			self._lazy_constraint[1] = cplex.SparsePair(ind=indices,val=values)
			self._lazy_rhs[1] = rhs
