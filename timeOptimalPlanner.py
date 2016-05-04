#!/usr/bin/env python
"""
=======================================================
timeOptimalPlanner.py - Finds the cost associated with the optimal plan for a mobile robot.
=======================================================
"""

import numpy as np
import sys, getopt, textwrap

def dijkstra(edgeCost):
	''' 
	Implements Dijkstra's algorithm.  The node with the lowest index is the initial node; the node with the highest index is the goal.

	Inputs:
		edgeCost : n-by-n dict of costs for each edge (pair of nodes), where n is the number of nodes in the graph.

	Outputs:
		minCost : (scalar) decimal value of the minimum cost of traversing the graph from the initial node to the final node.
	'''

	costToGo = {}
	nodeList = []

	# Initialize the list of nodes
	for i in edgeCost.keys():
		nodeList.append(i)
		costToGo[i] = np.inf

	# The list of keys are predecessors and thus only cover up to n-1 waypoints.  Account for the final n waypoint separately.
	nodeList.append(max(edgeCost.keys())+1)
	costToGo[max(edgeCost.keys())+1] = np.inf

	# Initialize the initial node with zero cost
	costToGo[0] = 0
	
	while True:
		# Find the dict keys corresponding to the edge with the smallest running cost so far
		i = min(costToGo,key=costToGo.get)

		# We are done with the cost-minimizing node from this point forward.
		nodeList.pop(nodeList.index(i))

		# Once all the nodes have been visited... we're done!
		if nodeList == []:
			break

		neighbors = [k for k in edgeCost[i].keys() if k in nodeList]
		for j in neighbors:
			# Loop over all "alive" neighbors to node i
			testCost = costToGo[i] + edgeCost[i][j]

			# Test if any running cost reduces the cost-to-go for the successor node; if so, overwrite that cost-to-go with the newly-computed cost.
			if testCost < costToGo[j]:
				costToGo[j] = testCost

		# Remove the cost-minimizing node from the cost here also.
		costToGo.pop(i)

	# Retrieve the cost at the goal node.
	minCost = costToGo[costToGo.keys()[0]]

	return minCost

def parseInput(fname):
	'''
	Let N (1 <= N <= 1000) be the number of waypoints on the course, without the first and last waypoints. 
	Parse an input file of a specific structure:
	- The file starts with an integer, N, representing the number of waypoints/costs in the list.
	- The following N lines describe a waypoint: three integers, X, Y and P, where (X, Y) is a location on the course 
	(1 <= X, Y <= 99, X and Y in meters) and P the penalty incurred the waypoint is missed (1 <= P <= 100).
	- This repeats for a given number of test cases (M); the end-of-file is indicated by a "0" as the entry for that line.

	Inputs:
		fname : A string containing the input .txt file name.

	Outputs:
		waypoints : An M-by-N-by-2 list containing an ordered list of N waypoints for M test cases.
		costs : An M-by-N list containing an ordered list of time penalties (P) for skipping a given waypoint.
	'''

	with open(fname,'r') as f:

		if f == []:
			raise Exception('no data supplied in the specified file.')

		# Parse the input file
		numWaypoints = []
		waypoints = [[]]
		costs = [[]]
		i = 0
		for line in f:
			# print line.split()
			if len(line.split()) == 0:
				pass
			elif int(line.split()[0]) == 0:
				break
			elif len(line.split()) == 1:
				# We've found a new test case
				i += 1
				numWaypoints.append(int(line.split()[0]))
				if i > 1:
					waypoints.append([])
					costs.append([])
			else:
				# Expect three integers (two x, y waypoints and one cost)
				a,b,c = [int(x) for x in line.split()]
				waypoints[i-1].append([a, b])
				costs[i-1].append(c)
				
	for i in range(len(numWaypoints)):
		if not len(waypoints[i]) == numWaypoints[i] or not len(costs[i]) == numWaypoints[i]:
			raise Exception("the provided lists of waypoints do not match the expected number")

	return waypoints, costs

def writeToFile(fname, minPathCostList):
	'''  
	Write the optimal costs to a file, one line for each of the M test cases.

	Inputs: 
		fname : A string containing the input .txt file name.  The output will be written to a file of the same name but with the .out extension.
	'''

	f = open(fname.strip('.txt')+'.out','w')
	for i in range(len(minPathCostList)):
		f.write(str(minPathCostList[i])+'\n') 
	f.close() 

def minimizePathCost(waypointList, costList, robotVelocity, dwellTime):
	'''  
	Construct an undirected graph with all combinations of waypoints visited, with the first and last waypoints being immutable 
	(they must be visited).  Labels on the edges correspond to the costs incurred for each transition.  

	Inputs: 
		waypointList : An M-by-N-by-2 list containing an ordered list of waypoints for M test cases.
		costList : An M-by-N list containing an ordered list of time penalties (P) for skipping a given waypoint.
		robotVelocity : (scalar) the velocity of the robot [m/s]
		dwellTime : (scalar) the time spent visiting each waypoint (excluding initialPosition) [sec]

	Outputs:
		minCostList : An M-dimensional list containing the optimal costs for of the M test cases. 
	'''

	numCases = len(waypointList)
	minCostList = [0] * numCases

	for i in range(numCases):
		# i is the current test case
		
		# print i
		edgeCost = {}
		numWaypoints = len(waypointList[i])

		for j in range(numWaypoints - 1):
			# j is the predecessor node

			edgeCost[j] = {}
			nextWaypointsIndexSet = list(set(range(numWaypoints)) - set(range(j+1)))

			for k in nextWaypointsIndexSet:
				# k is the successor node

				# Determine the cost of skipping any intervening waypoints in the ordered sequence
				skippedWaypoints = range(j+1,k)
				skippingCost = sum([costList[i][l-1] for l in skippedWaypoints])

				# Determine the travel cost, including the time spent at each waypoint, assuming straight-line travel between waypoints
				distanceXY = np.array(waypointList[i][k]) - np.array(waypointList[i][j])
				distance = np.sqrt(pow(distanceXY[0],2) + pow(distanceXY[1],2))
				travelCost = distance/robotVelocity + dwellTime

				# Compute the total cost incurred on this edge
				edgeCost[j][k] = travelCost + skippingCost

		# Run Dijkstra's algorithm
		minCostList[i] = dijkstra(edgeCost)

	return minCostList

def main(fname, initPosition, finalPosition, robotVelocity, dwellTime):
	''' 
	This function reads waypoints and costs  an input file, adds the pre-configured initial and final waypoints to the list, and finds an 
	optimal strategy for the given waypoint sequence, and cost penalties for skipping waypoints, and writes the optimal cost (time expenditure) 
	to a data file.

	Inputs: 
		fname : A string containing the input .txt file name.  The output will be written to a file of the same name but with the .out extension.
		initPosition : a 2-dimensional list containing the initial x-y-position of the robot in the factory.
		finalPosition : a 2-dimensional list containing the desired final (goal) x-y-position of the robot in the factory.
		robotVelocity : (scalar) the velocity of the robot [m/s]
		dwellTime : (scalar) the time spent visiting each waypoint (excluding initialPosition) [sec]
	'''

	# Obtain the ordered list of waypoints for each test case and the time penalties for skipping these waypoints
	waypointList, costList = parseInput(fname)

	# Prepend and append the fixed initial and final conditions
	for i in range(len(waypointList)):
		waypointList[i].insert(0,initPosition)
		waypointList[i].append(finalPosition)
		
	# Find the optimal strategy for the given (ordered) set of waypoints
	minPathCostList = minimizePathCost(waypointList, costList, robotVelocity, dwellTime)

	# Round the result to three significant digits
	roundedMinPathCost = np.round(minPathCostList,3)

	# Write the result to an output file
	writeToFile(fname, roundedMinPathCost)

if __name__ == '__main__':

	# Set defaults
	filename 		= 'sample.txt'
	initPosition 	= [0, 0] 		# meters
	finalPosition 	= [100, 100]  	# meters
	robotVelocity	= 2				# m/s
	dwellTime		= 10			# sec

	# Process the optional command-line arguments
	try:
		opts, args = getopt.getopt(sys.argv[1:], "hf:ip:fp:v:d:", ["help", "file=", "init-pos=", "final-pos=", "vel=", "dwell-time="])
	except getopt.GetoptError:
		raise Exception("Bad arguments")
		sys.exit(2)

	for opt, arg in opts:
		if opt in ("-h", "--help"):
			print textwrap.dedent("""\
						Usage: timeOptimalPlanner.py [-h] [-f <file name>] [-ip <X0,Y0>] [-fp <Xf,Yf>] [-v <V>] [-d <D>]

						-h, --help:
							Display this help message
						-f, --file FILE:
							Name of the input file in .txt format
							Default: sample.txt
						-ip, --init-pos X0,Y0:
							An initial position, X0, Y0 (in meters)
							Default: 0,0 
						-fp, --final-pos XF,YF:
							A final position, Xf, Yf (in meters)
							Default: 100,100 
						-v, --vel V:
							The robot's velocity, V (in meters/sec)
							Default: 2
						-d, --dwell-time D:
							The required dwell time spent at each waypoint, D (in seconds)
							Default: 10""")

			sys.exit()
		elif opt in ("-f", "--file"):
			filename = arg
		elif opt in ("-ip", "--init-pos"):
			ip = []
			a = arg.split(',')
			for i in a:
				ip.append(int(i))
			initialPosition = ip
		elif opt in ("-fp", "--final-pos"):
			fp = []
			a = arg.split(',')
			for i in a:
				fp.append(int(i))
			finalPosition = fp
		elif opt in ("-v", "--vel"):
			robotVelocity = float(arg)
		elif opt in ("-t", "--dwell-time"):
			dwellTime = float(arg)

	main(filename, initPosition, finalPosition, robotVelocity, dwellTime)