import cv2
import math
import numpy as np
from scipy.linalg import solve
from sklearn.cluster import KMeans

maze = []

def findSpacing(radii):

	diff1 = [radii[i+1] - radii[i] for i in range(len(radii)-1)]
	diff2 = [radii[i+2] - radii[i] for i in range(len(radii)-2)]
	diff3 = [radii[i+3] - radii[i] for i in range(len(radii)-3)]
	diff4 = [radii[i+4] - radii[i] for i in range(len(radii)-4)]
	diff5 = [radii[i+5] - radii[i] for i in range(len(radii)-5)]

	diffs = np.concatenate([diff1, diff2, diff3, diff4, diff5]).reshape(-1,1)

	base = radii[0]

	spacing = 0
	score = 1000000
	k = 3
	searching = True

	while searching:
		means = KMeans(k).fit(diffs)

		mCenters = [c[0] for c in means.cluster_centers_]
		mCenters.sort()

		# print(mCenters)
		# print(means.inertia_)

		centerDiffs = np.asarray([mCenters[i+1] - mCenters[i] for i in range(k-1)]).reshape(-1,1)
		centerMeans = KMeans(2).fit(centerDiffs)

		mCenters2 = [c[0] for c in centerMeans.cluster_centers_]
		mCenters2.sort()

		# print(mCenters2)
		# print(centerMeans.inertia_)

		if (mCenters2[0] < 4) or (k > len(radii)/2):
			searching = False
			break

		# print(mCenters2)

		mScore = means.inertia_ + centerMeans.inertia_
		# print(k, mScore, mCenters2[0])

		if mScore < score:
			score = mScore
			spacing = mCenters2[0]

		k+=1
		# if k > 10:
		# 	searching = False

	print("spacing: " + str(spacing))
	return(spacing)

def calculateScore(radii, numLines):
	minimum = radii[0]
	diff = radii[-1] - minimum
	# print(minimum, diff)

	spacing = diff/(numLines - 1)
	# print(numLines, spacing)

	score = 0
	for r in radii:
		score += math.cos(2*np.pi*(r-minimum)/spacing)
	# print(numLines, score)
	return score

def findNumLines(radii):
	# print(radii)

	numLines = int(len(radii)*0.8)
	# print numLines

	best = numLines
	score = calculateScore(radii, numLines)

	done = False
	while not done:
		numLines+=1
		newScore = calculateScore(radii, numLines)

		if score > 0.5*len(radii):
			if newScore < score:
				done = True

		if numLines > 5*len(radii):
			print("Not enought lines detected to interpolate")
			done = True
		# print(newScore, numLines)

		if newScore > score:
			score = newScore
			best = numLines

	# print(best)
	return best

def generateSequence(start, stop, num):
	diff = stop - start

	points = []
	for i in range(num):
		# print(i, num)
		points.append(start + (diff*i)/(num-1))

	# points = [(start + (diff*((i*1.0)/(num)))) for i in range(num)]
	return(points)

def lineIntersect(r1,c1,s1,r2,c2,s2):
	xs1 = c1*r1
	ys1 = s1*r1

	xs2 = c2*r2
	ys2 = s2*r2

	A = [[s1,-s2],[-c1,c2]]
	b = [xs1 - xs2, ys1 - ys2]

	x = solve(A,b)

	# print(xs1-s1*x[0], ys1+c1*x[0])
	# print(xs2-s2*x[1], ys2+c2*x[1])

	#r1*c1 - a*s1 = r2*c2 - b*s2
	#r1*s1 + a*c1 = r2*s2 + b*c2

	#r1*c1 - r2*c2 = a*s1 - b*s2
	#r1*s1 - r2*s2 = -a*c1 + b*c2

	#[s1,-s2] * [a] = r1*c1 - r2*c2
	#[-c1,c2] * [b] = r1*s1 - r2*s2

	return([xs1-s1*x[0], ys1+c1*x[0]])

def getMaze():
	main()
	return maze

def main():

	raw = cv2.imread('testMaze2.png')
	# cv2.imshow("raw", raw)

	grey = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
	# cv2.imshow("grey", grey)

	edges = cv2.Canny(grey, 50, 150)
	# cv2.imshow("edges",edges)

	lines = cv2.HoughLines(edges,1,np.pi/180,200)

	result = raw.copy()
	linesFound = 0

	group1Lines = []
	group1Count = 0
	group1TSum = 0.0

	group2Lines = []
	group2Count = 0
	group2TSum = 0

	for pair in lines:
		# print("====="*20)
		r = pair[0][0]
		t = pair[0][1]%np.pi

		avg1a = 0
		if group1Count > 0:
			avg1a = (group1TSum/group1Count)%np.pi

		avg2a = (avg1a + (np.pi/2))%np.pi
		if group2Count > 0:
			avg2a = (group2TSum/group2Count)%np.pi
			if group1Count == 0:
				avg1a = (avg2a + (np.pi/2))%np.pi

		avg1b = avg1a+np.pi
		avg2b = avg2a+np.pi

		d1a = abs(avg1a-t)
		d2a = abs(avg2a-t)
		d1b = abs(avg1b-t)
		d2b = abs(avg2b-t)

		selection = 1
		best = d1a

		if d2a < best:
			best = d2a
			selection = 2

		if d1b < best:
			best = d1b
			selection = 1

		if d2b < best:
			best = d2b
			selection = 2

		if selection == 1:
			group1Lines.append(pair[0])
			group1Count +=1
			group1TSum += t
			# print(r, t, 1)
		else:
			group2Lines.append(pair[0])
			group2Count +=1
			group2TSum += t
			# print(r, t, 2)

		co = np.cos(t)
		si = np.sin(t)

		x0 = co*r
		y0 = si*r

		x1 = int(x0 - 1000*si)
		y1 = int(y0 + 1000*co)
		x2 = int(x0 + 1000*si)
		y2 = int(y0 - 1000*co)


		if r < 50 and t > 1:
			print(r,t)
		cv2.line(result, (x1, y1), (x2, y2), (255,0,0), 2)
		linesFound +=1

	print("Num lines found: " + str(linesFound))

	# print(group1Count, group2Count)
	# print(group1TSum/group1Count, group2TSum/group2Count)

	group1Lines.sort(key=lambda line: line[0])
	group2Lines.sort(key=lambda line: line[0])

	radii1 = [r for r,t in group1Lines]
	radii2 = [r for r,t in group2Lines]

	# spacing1 = findSpacing(radii1)
	# spacing2 = findSpacing(radii2)

	numLines1 = findNumLines(radii1)
	numLines2 = findNumLines(radii2)

	print((radii1[-1]-radii1[0])/(numLines1-1), (radii2[-1]-radii2[0])/(numLines2-1))

	# print(radii1)
	# print(radii1[-1])

	sequence1 = generateSequence(radii1[0], radii1[-1], numLines1)
	sequence2 = generateSequence(radii2[0], radii2[-1], numLines2)

	# print(sequence1)

	t1 = group1TSum / group1Count
	t2 = group2TSum / group2Count

	print(t1,t2)

	c1 = np.cos(t1)
	s1 = np.sin(t1)
	c2 = np.cos(t2)
	s2 = np.sin(t2)

	for r in sequence1:
		x0 = c1*r
		y0 = s1*r

		x1 = int(x0 - 1000*s1)
		y1 = int(y0 + 1000*c1)
		x2 = int(x0 + 1000*s1)
		y2 = int(y0 - 1000*c1)

		if r < 50 and t1 > 1:
			print(r,t1)

		cv2.line(result, (x1, y1), (x2, y2), (0,0,255), 1)

	print (group2Count, len(sequence2))

	for r in sequence2:
		x0 = c2*r
		y0 = s2*r

		x1 = int(x0 - 1000*s2)
		y1 = int(y0 + 1000*c2)
		x2 = int(x0 + 1000*s2)
		y2 = int(y0 - 1000*c2)

		cv2.line(result, (x1, y1), (x2, y2), (0,0,255), 1)

	junctions = []

	grid = raw.copy()	
	
	for a in sequence1:
		row = []
		for b in sequence2:
			row.append(lineIntersect(a,c1,s1,b,c2,s2))
			cv2.circle(grid, (int(row[-1][0]), int(row[-1][1])), 1, (0,255,0), 2)
		junctions.append(row)

	cells = []

	contour1 = np.asarray([junctions[0][0], junctions[0][1], junctions[1][1], junctions[1][0]], dtype=np.int32)
	contour2 = np.asarray([junctions[1][0], junctions[1][1], junctions[2][1], junctions[2][0]], dtype=np.int32)

	# print(grey.shape)
	mask = np.zeros(grey.shape, dtype=np.uint8)

	# cv2.imshow("mask", mask)

	for i in range(len(sequence1)-1):
		row = []
		for j in range(len(sequence2)-1):
			contour = np.asarray([junctions[i][j], junctions[i][j+1], junctions[i+1][j+1], junctions[i+1][j]], dtype=np.int32)

			cv2.drawContours(mask,[contour], -1, 255, -1)
			mean = cv2.mean(raw, mask=mask)
			cv2.drawContours(mask,[contour], -1, 0, -1)
			if (mean[0] > 127) and (mean[1] > 127) and (mean[2] > 127):
				row.append(0)
			else:
				row.append(1)


			if i == 5 and j == 6:
				cv2.drawContours(grid, [contour], -1, (0,255,0), 1)
				print(mean)
		cells.append(row)

	# cv2.drawContours(grid, [contour1, contour2], -1, (0,255,0), 1)

	print(cells)

	maze = cells

	# cv2.imshow("res", result)
	cv2.imshow("grid", grid)
	cv2.waitKey(-1)
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
