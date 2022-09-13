import cplex
import os
from GTObj import GameTheoryObj as obj
import pickle

def extract(filename):
	exp = []
	with open(filename, "r") as fin:
		f = fin.readlines()
		for aLine in f:
			temp = ""
			aLine = aLine.split("[")[1:]
			ind = aLine[0].split("]")[0].split(",")
			val = aLine[1].split("]")[0].split(",")
			for i in range(len(ind)):
				index = ind[i]
				value = val[i]
				if value == "1":
					temp = temp + convertInd(index) + "+"
				else:
					temp = temp + "(1-" + convertInd(index) + ")+"
			exp.append(temp[:-1])
	with open("masterConstraint.txt", "w") as fout:
		for anExp in exp:
			fout.write(anExp + ">=1\n")
			
def convertInd(index):
	switcher={
		"0": "x0",
		"1": "x1",
		"2": "x2",
		"3": "y0",
		"4": "y1",
		"5": "y2"
	}
	return switcher.get(index, "nothing")

def disableOutput(model):
	model.set_log_stream(None)
	model.set_error_stream(None)
	model.set_warning_stream(None)
	model.set_results_stream(None)

def createInstance(x, y, num):
	folderName = "Instance/%d-%d/%d-%d-%d-%d" % (x[0], y[0], x[1][0],x[1][1], y[1][0], y[1][1])
	if not os.path.exists(folderName):
		os.makedirs(folderName)
	for i in range(num):
		instance = obj(x, y)
		with open("%s/ins%d.obj" % (folderName, i), "wb") as fout:
			pickle.dump(instance, fout, protocol=2)
		

def loadInstance(x, y):
	folderName = "Instance/%d-%d/%d-%d-%d-%d" % (x[0], y[0], x[1][0],x[1][1], y[1][0], y[1][1])
	instances = []

	objFiles = [f for f in os.listdir(folderName) if f[-4:] == ".obj"]
	print(objFiles)

	for i in range(len(objFiles)):
		objFile = "ins%d.obj" % i
		with open("%s/%s" % (folderName, objFile), "rb") as fin:
			instance = pickle.load(fin, encoding='latin1')
			instances.append(instance)

	return instances