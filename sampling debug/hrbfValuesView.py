from matplotlib import pyplot
import pylab
from mpl_toolkits.mplot3d import Axes3D
import random

#hrbf_file = open("left_leg_HRBF")
#hrbf_file = open("hips_HRBF")
#hrbf_file = open("left_leg_grad_HRBF")
#hrbf_file = open("composition_rest_dev5")
#hrbf_file = open("composition_rest_full")
hrbf_file = open("composition_pose_full")

sample_pts_x = []
sample_pts_y = []
sample_pts_z = []

bounding_x = []
bounding_y = []
bounding_z = []

fig = pylab.figure()
ax = Axes3D(fig)

numEntries = 0

for line in hrbf_file:


	numEntries += 1
	vals = line.split()
	val = float(vals[3])
	y = float(vals[1])
	if val >= 999.0:
		bounding_x.append(float(vals[0]))
		bounding_y.append(float(vals[1]))
		bounding_z.append(float(vals[2]))
		continue

	# randomly discard some percentage of entries to help us out
	dice = random.random()
	if dice > 0.01: continue # discard 99% of the data for now

	if val >= 0.0001:#and y > 0.0: # slide cutoff to view gradient change
		sample_pts_x.append(float(vals[0]))
		sample_pts_y.append(float(vals[1]))
		sample_pts_z.append(float(vals[2]))

# make a nice square AABB for drawing.
distX = abs(bounding_x[0] - bounding_x[1])
distY = abs(bounding_y[0] - bounding_y[1])
distZ = abs(bounding_z[0] - bounding_z[1])

centX = (bounding_x[0] + bounding_x[1]) / 2
centY = (bounding_y[0] + bounding_y[1]) / 2
centZ = (bounding_z[0] + bounding_z[1]) / 2

aaCubeRad = max(max(distX, distY), distZ) / 2

aaCubeX = [centX - aaCubeRad, centX + aaCubeRad]
aaCubeY = [centY - aaCubeRad, centY + aaCubeRad]
aaCubeZ = [centZ - aaCubeRad, centZ + aaCubeRad]

dred = (0.5, 0.0, 0.0)
lred = (1.0, 0.5, 0.5)
dblue = (0.0, 0.0, 0.5)
lblue = (0.5, 0.5, 1.0)
green = (0.0, 1.0, 0.0)

white = (1.0, 1.0, 1.0)

print("num entries: " + str(numEntries))

ax.scatter(sample_pts_x, sample_pts_y, sample_pts_z, c=lblue)
ax.scatter(bounding_x, bounding_y, bounding_z, c=lred)

ax.scatter(aaCubeX, aaCubeY, aaCubeZ, c=white)

ax.scatter([0], [0], [0], c=green)

pyplot.show()