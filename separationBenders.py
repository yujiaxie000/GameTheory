import cplex
from cplex.callbacks import LazyConstraintCallback, BranchCallback
from util import *
from addPrimal import *
import numpy as np
import sys
from cplex.exceptions.errors import CplexSolverError

class MySeparationBenders():
	def __init__(self, instance):
		self._bigM = self.initBigM(instance)
		self._modelX, self._modelY = self.initSeparationBenders(instance)
		self._lazy_constraint = [0, 0]
		self._lazy_rhs = [0, 0]

	def initBigM(self, instance):
		return {'x': 10000, 'y': 10000}

	def initSeparationBenders(self, instance):
		modelX = self.initSeparationHelper(instance, "x")
		modelY = self.initSeparationHelper(instance, "y")
		return modelX, modelY

	def initSeparationHelper(self, instance, attr):
		if attr == "x":
			num_self = instance._xNum
			num_other = instance._yNum
			attr_other = "y"
		elif attr == "y":
			num_self = instance._yNum
			num_other = instance._xNum
			attr_other = "x"
		else:
			return

		model = cplex.Cplex()
		lb = []
		ub = []
		objective = []
		variableName = []

		# 4b
		for idx in range(num_self):
			lb.append(-cplex.infinity)
			ub.append(cplex.infinity)
			objective.append(0.0)
			variableName.append("r1%s%d" % (attr, idx))

		# 4c
		lb.append(-cplex.infinity)
		ub.append(cplex.infinity)
		# v_type += "C"
		objective.append(1.0)
		variableName.append("r2%s" % attr)

		# 4d
		for idx in range(num_other):
			lb.append(0.0)
			ub.append(cplex.infinity)
			# v_type += "C"
			objective.append(0.0) # TO BE MODIFIED
			variableName.append("r3%s%d" % (attr, idx))

		# 4e
		for idx in range(num_self):
			lb.append(0.0)
			ub.append(cplex.infinity)
			# v_type += "C"
			objective.append(self._bigM[attr]) # TO BE MODIFIFED
			variableName.append("r4%s%d" % (attr, idx))

		model.objective.set_sense(model.objective.sense.minimize)
		model.variables.add(names=variableName, lb=lb, ub=ub, obj=objective)

		counter = 0
		idx_4b = [idx + counter for idx in range(num_self)]
		counter += len(idx_4b)
		idx_4c = counter
		counter += 1
		idx_4d = [idx + counter for idx in range(num_self)]
		counter += len(idx_4d)
		idx_4e = [idx + counter for idx in range(num_self)]

		matrix = []
		rhs = []
		constraint_name = []
		constraint_senses = []

		# p_other
		for idx in range(num_other):
			values_4b = self.getSumHelper(instance, idx, attr)
			values = values_4b + [1.0, 1.0]
			indices = idx_4b + [idx_4c, idx_4d[idx]]
			matrix.append(cplex.SparsePair(ind=indices, val=values))
			rhs.append(0.0)
			constraint_senses.append("G")
			constraint_name.append("p%s%d" % (attr_other, idx))

		# s
		for idx in range(num_self):
			matrix.append(cplex.SparsePair(ind=[idx_4b[idx], idx_4e[idx]], val=[1.0, 1.0]))
			rhs.append(0.0)
			constraint_senses.append("G")
			constraint_name.append("s%s%d" % (attr, idx))

		# v
		matrix.append(cplex.SparsePair(ind=idx_4b, val=[-1.0 for x in idx_4b]))
		rhs.append(1.0) # NOT CONE
		constraint_senses.append("E")
		constraint_name.append("v%s" % attr)

		model.linear_constraints.add(rhs=rhs, lin_expr=matrix, names=constraint_name, senses=constraint_senses)
		return model

	def getSumHelper(self, instance, idx, attr):
		values = []
		if attr == "x":
			for i in range(instance._xNum):
				values.append(instance._value[i][idx]['x'])
		elif attr == "y":
			for j in range(instance._yNum):
				values.append(instance._value[idx][j]['y'])
		return values


	def separateXBenders(self, supports, instance):
		supportX, supportY, supportX_M, supportY_M = supports
		model = cplex.Cplex(self._modelX)
		# model.write("model_dualX_init.lp")
		for idx in supportY:
			model.objective.set_linear("r3x%d" % idx, 1.0)
		for idx in supportX:
			model.objective.set_linear("r4x%d" % idx, 0.0)
		model.parameters.preprocessing.presolve.set(model.parameters.preprocessing.presolve.values.off)
		disableOutput(model)
		model.solve()
		return model

	def separateYBenders(self, supports, instance):
		supportX, supportY, supportX_M, supportY_M = supports
		model = cplex.Cplex(self._modelY)
		# model.write("model_dualY_init.lp")
		for idx in supportX:
			model.objective.set_linear("r3y%d" % idx, 1.0)
		for idx in supportY:
			model.objective.set_linear("r4y%d" % idx, 0.0)
		model.parameters.preprocessing.presolve.set(model.parameters.preprocessing.presolve.values.off)
		disableOutput(model)
		model.solve()
		return model

	def separate(self, supports, instance, zx, zy, epsilon=1e-5):
		self._lazy_constraint = [0, 0]
		self._lazy_rhs = [0, 0]
		modelX = self.separateXBenders(supports, instance)
		modelY = self.separateYBenders(supports, instance)
		modelX_status = modelX.solution.get_status()
		modelY_status = modelY.solution.get_status()
		if modelX_status == 1 and modelY_status == 1:
			self.constructLazyConstraint(modelX, modelY, supports, instance, zx, zy, epsilon)
		elif modelX_status == 2 or modelY_status == 2:
			self.constructLazyConstraint(modelX, modelY, supports, instance, zx, zy, epsilon)
		return modelX_status, modelY_status

	def constructLazyConstraint(self, modelX, modelY, supports, instance, zx, zy, epsilon):
		supportX, supportY, supportX_M, supportY_M = supports
		modelX_status = modelX.solution.get_status()
		modelY_status = modelY.solution.get_status()

		if modelX_status == 1 and modelY_status == 1:
			#x
			if zx > modelX.solution.get_objective_value() + epsilon:

				indices = [idx for idx in range(instance._xNum + instance._yNum)]
				indices.append(instance._xNum + instance._yNum)
				values, rhs = self.lazyConstraintHelper(instance, modelX.solution.get_values(), 'x')
				values.append(-1.0)
				self._lazy_constraint[0] = cplex.SparsePair(ind=indices, val=values)
				self._lazy_rhs[0] = rhs

			#y
			if zy > modelY.solution.get_objective_value() + epsilon:
				indices = [idx for idx in range(instance._xNum + instance._yNum)]
				indices.append(instance._xNum + instance._yNum + 1)
				values, rhs = self.lazyConstraintHelper(instance, modelY.solution.get_values(), 'y')
				values.append(-1.0)
				self._lazy_constraint[1] = cplex.SparsePair(ind=indices, val=values)
				self._lazy_rhs[1] = rhs

		if modelX_status == 2: # 
			indices = [idx for idx in range(instance._xNum + instance._yNum)]
			values, rhs = self.lazyConstraintHelper(instance, modelX.solution.advanced.get_ray(), 'x')

			self._lazy_constraint[0] = cplex.SparsePair(ind=indices, val=values)
			self._lazy_rhs[0] = rhs

		if modelY_status == 2:
			indices = [idx for idx in range(instance._xNum + instance._yNum)]
			values, rhs = self.lazyConstraintHelper(instance, modelY.solution.advanced.get_ray(), 'y')
			self._lazy_constraint[1] = cplex.SparsePair(ind=indices, val=values)
			self._lazy_rhs[1] = rhs

	def lazyConstraintHelper(self, instance, solutions, attr):

		if attr != "x" and attr != "y":
			return
		if attr == "x":
			num_self = instance._xNum
			num_other = instance._yNum
		else:
			num_self = instance._yNum
			num_other = instance._yNum

		counter = num_self
		idx_4c = counter
		counter += 1
		idx_4d = [idx + counter for idx in range(num_other)]
		counter += len(idx_4d)
		idx_4e = [idx + counter for idx in range(num_self)]

		values = []
		rhs = -1 * solutions[idx_4c]
		if attr == "x":
			for idx in range(instance._xNum):
				values.append(-1 * self._bigM[attr] * solutions[idx_4e[idx]])
				rhs -= self._bigM[attr] * solutions[idx_4e[idx]]
			for idx in range(instance._yNum):
				values.append(solutions[idx_4d[idx]])
		else:
			for idx in range(instance._xNum):
				values.append(solutions[idx_4d[idx]])
			for idx in range(instance._yNum):
				values.append(-1 * self._bigM[attr] * solutions[idx_4e[idx]])
				rhs -= self._bigM[attr] * solutions[idx_4e[idx]]
		return values, rhs
