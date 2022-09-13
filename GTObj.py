import random
import csv
import numpy as np

class GameTheoryObj(object):
	def __init__(self, x, y):
		self._XRange = [np.inf, -np.inf]
		self._YRange = [np.inf, -np.inf]
		self._value = self.generate(x, y)
		self._xNum = x[0]
		self._yNum = y[0]
		# self.writeReward()
		self._X, self._Y = self.toNpArray()
		self._maxX, self._maxY, self._maxV = self.getMax()
		
		# print(self._X)
		# print(self._Y)

	def get_nonDominated(self, action, mode):
		if mode == 'x':
			reduceMatrix = np.take(self._Y, action, axis=0)
			dominated = np.zeros(self._yNum)
			for j in range(self._yNum):
				for jj in range(j + 1, self._yNum, 1):
					if (reduceMatrix[:,j] <= reduceMatrix[:,jj]).all():
						dominated[j] = 1
						break
			
			current = np.argwhere(dominated == 0).flatten()
			for j in current:

				if (reduceMatrix[:,-1] <= reduceMatrix[:,j]).all():
					if j != self._yNum - 1:
						dominated[-1] = 1
			return np.argwhere(dominated == 0).flatten()

		else:
			reduceMatrix = np.take(self._X, action, axis=1)
			dominated = np.zeros(self._xNum)
			for i in range(self._xNum):
				for ii in range(i + 1, self._xNum, 1):
					if (reduceMatrix[i] <= reduceMatrix[ii]).all():
						dominated[i] = 1
						break
			
			current = np.argwhere(dominated == 0).flatten()
			for i in current:

				if (reduceMatrix[-1] <= reduceMatrix[i]).all():
					if i != self._xNum - 1:
						dominated[-1] = 1

			return np.argwhere(dominated == 0).flatten()

	def generate(self, x, y):
		xNum, xRange = x
		yNum, yRange = y
		aDict = {}
		for i in range(xNum):
			aDict[i] = {}
			for j in range(yNum):
				aDict[i][j] = {}
				# aDict[i][j]['x'] = round(random.uniform(xRange[0], xRange[1]), 2)
				# aDict[i][j]['y'] = round(random.uniform(yRange[0], yRange[1]), 2)
				aDict[i][j]['x'] = round(random.uniform(xRange[0], xRange[1]), 2)
				aDict[i][j]['y'] = round(random.uniform(yRange[0], yRange[1]), 2)
				#print(random.uniform(xRange[0], xRange[1]))
				self._XRange = [min(self._XRange[0], aDict[i][j]['x']), max(self._XRange[1], aDict[i][j]['x'])]
				self._YRange = [min(self._YRange[0], aDict[i][j]['y']), max(self._YRange[1], aDict[i][j]['y'])]
		#
		return aDict

	def getMax(self):
		return float(np.amax(self._X)), float(np.amax(self._Y)), float(np.amax(self._X + self._Y))
		#return 400, 500, 900

	def getMaxEqlm(self):
		# row
		RR,RC = self._X.shape
		x_idx = []
		for i in range(RC):
			columnAction = self._X[:,i].flatten()
			indices = np.argwhere(columnAction == np.max(columnAction))
			for aMax in indices.flatten('C'):
				x_idx.append((aMax, i))

		# column
		CR, CC = self._Y.shape
		y_idx = []
		for j in range(CR):
			rowAction = self._Y[j].flatten()
			indices = np.argwhere(rowAction == np.max(rowAction))
			for aMax in indices.flatten('C'):
				y_idx.append((j, aMax))


		match = np.array([x for x in x_idx for y in y_idx if np.array_equal(x,y)])

		maxObj = 0
		xMax = 0.0
		yMax = 0.0
		maxIdx = None
		# print(self._X)
		# print(self._Y)
		for i,j in match:
			curr = self._X[i][j] + self._Y[i][j]
			if curr > maxObj:
				maxObj = curr
				xMax = self._X[i][j]
				yMax = self._Y[i][j]
				maxIdx = [i,j]
		return float(xMax), float(yMax), maxIdx
		#return 10.0, 10.0, [1,1]

	def toNpArray(self):
		X = []
		Y = []
		for i in range(self._xNum):
			tempX = []
			tempY = []
			for j in range(self._yNum):
				tempX.append(self._value[i][j]['x'])
				tempY.append(self._value[i][j]['y'])
			X.append(tempX)
			Y.append(tempY)
		X = np.array(X)
		Y = np.array(Y)
		return X, Y

	def writeReward(self):
		rowsX = []
		rowsY = []
		for i in range(self._xNum):
			rowsX.append([])
			rowsY.append([])
			for j in range(self._yNum):
				rowsX[i].append(self._value[i][j]['x'])
				rowsY[i].append(self._value[i][j]['y'])
		with open("Reward/rewardX.csv", "w") as fout:
			csvWriter = csv.writer(fout)
			csvWriter.writerows(rowsX)

		with open("Reward/rewardY.csv", "w") as fout:
			csvWriter = csv.writer(fout)
			csvWriter.writerows(rowsY)

	def getIndexX(self, i):
		return i

	def getIndexY(self, j):
		return j + self._xNum

	def getIndicesX(self, xIndices):
		return xIndices

	def getIndicesY(self, yIndices):
		return [j + self._xNum for j in yIndices]

	def getMissingX(self, xIndices):
		missing = [i for i in range(self._xNum) if i not in xIndices]
		return missing

	def getMissingY(self, yIndices):
		yIndices = self.getIndicesY(yIndices)
		missing = [j for j in range(self._xNum, self._xNum + self._yNum, 1) if j not in yIndices]
		return missing

	def convert(self, merged):
		X = []
		X_M = []
		Y = []
		Y_M = []

		for i in range(self._xNum):
			if merged[i] > 0:
				X.append(i)
			else:
				X_M.append(i)

		for j in range(self._xNum, self._xNum + self._yNum, 1):
			if merged[j] > 0:
				Y.append(j - self._xNum)
			else:
				Y_M.append(j - self._xNum)

		return X, Y, X_M, Y_M

	def convertReducedX(self, lb, ub):
		X = []
		X_M = []
		Y = []
		Y_M = []

		for i in range(self._xNum):
			if abs(lb[i] - 1) < 0.000001 and abs(ub[i] - 1) < 0.000001:
				X.append(i)
			else:
				X_M.append(i)

		for j in range(self._xNum, self._xNum + self._yNum, 1):
			if abs(lb[j]) < 0.000001 and abs(ub[j]) < 0.000001:
				Y_M.append(j - self._xNum)
			else:
				Y.append(j - self._xNum)
				
		return X, Y, X_M, Y_M

	def convertReducedY(self, lb, ub):
		X = []
		X_M = []
		Y = []
		Y_M = []

		for i in range(self._xNum):
			if abs(lb[i]) < 0.000001 and abs(ub[i]) < 0.000001:
				X_M.append(i)
			else:
				X.append(i)

		for j in range(self._xNum, self._xNum + self._yNum, 1):
			if abs(lb[j] - 1) < 0.000001 and abs(ub[j] - 1) < 0.000001:
				Y.append(j - self._xNum)
			else:
				Y_M.append(j - self._xNum)

		return X, Y, X_M, Y_M


	def getSystemX(self, ub, lb):
		u_X, u_Y, u_XM, u_YM = ub
		l_X, l_Y, l_XM, l_YM = lb
		X = list(set(u_X) & set(l_X)) # X == 1
		Y_M = list(set(u_YM) & set(l_YM)) # Y == 0
		X_M = [i for i in range(self._xNum) if i not in X]
		Y = [j for j in range(self._yNum) if j not in Y_M]
		return X, X_M, Y, Y_M

	def getSystemY(self, ub, lb):
		u_X, u_Y, u_XM, u_YM = ub
		l_X, l_Y, l_XM, l_YM = lb
		Y = list(set(u_Y) & set(l_Y)) # Y == 1
		X_M = list(set(u_XM) & set(l_XM)) # X == 0
		Y_M = [j for j in range(self._yNum) if j not in Y]
		X = [i for i in range(self._xNum) if i not in X_M]
		return X, X_M, Y, Y_M

	def getSystems(self, ub, lb):
		X_X = []
		X_XM = []
		X_Y = []
		X_YM = []

		Y_X = []
		Y_XM = []
		Y_Y = []
		Y_YM = []

		for i in range(len(ub)):
			if i < self._xNum:
				if lb[i] == 1 and ub[i] == 1:
					X_X.append(i)
				elif lb[i] == 0 and ub[i] == 0:
					Y_XM.append(i)
			else:
				if lb[i] == 1 and ub[i] == 1:
					Y_Y.append(i - self._xNum)
				elif lb[i] == 0 and ub[i] == 0:
					X_YM.append(i - self._xNum)

		for i in range(self._xNum):
			if i not in X_X:
				X_XM.append(i)
			if i not in Y_XM:
				Y_X.append(i)

		for j in range(self._yNum):
			if j not in Y_Y:
				Y_YM.append(j)
			if j not in X_YM:
				X_Y.append(j)
		systemX = X_X, X_XM, X_Y, X_YM
		systemY = Y_X, Y_XM, Y_Y, Y_YM
		return systemX,systemY

	def getCorrespondingAction(self, p):
		p = np.array(p)
		p = p.reshape((self._xNum, self._yNum))

		corresponding = []
		for i in range(self._xNum):
			if np.sum(p[i]) == 0:
				corresponding.append(0)
			else:
				corresponding.append(1)
		for j in range(self._yNum):
			if np.sum(p[:,j]) == 0:
				corresponding.append(0)
			else:
				corresponding.append(1)
		return np.array(corresponding)
