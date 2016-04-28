from matplotlib import pyplot
import pylab
from mpl_toolkits.mplot3d import Axes3D

#hrbf_file = open("left_leg_samples")
#hrbf_file = open("hips_samples")
#hrbf_file = open("left_up_leg_samples")
#hrbf_file = open("neck_downsampled")
hrbf_file = open("left_leg_downsampled")


sample_pts_x = []
sample_pts_y = []
sample_pts_z = []

sample_nors_x = []
sample_nors_y = []
sample_nors_z = []

bounds = [40.0, -40.0]

extr_pts_x = []
extr_pts_y = []
extr_pts_z = []

extr_nors_x = []
extr_nors_y = []
extr_nors_z = []

fig = pylab.figure()
ax = Axes3D(fig)

numEntries = 0

for line in hrbf_file:
	numEntries += 1
	vals = line.split()
	val = int(float(vals[3]) + 0.01)
	if val == 1: # sample point 4 4 4
		sample_pts_x.append(float(vals[0]))
		sample_pts_y.append(float(vals[1]))
		sample_pts_z.append(float(vals[2]))
	elif val == 2: # sample nor
		sample_nors_x.append(float(vals[0]))
		sample_nors_y.append(float(vals[1]))
		sample_nors_z.append(float(vals[2]))
	elif val == 3: # extr point
		extr_pts_x.append(float(vals[0]))
		extr_pts_y.append(float(vals[1]))
		extr_pts_z.append(float(vals[2]))
		print("extremity point")
	elif val == 4: # extr nor
		extr_nors_x.append(float(vals[0]))
		extr_nors_y.append(float(vals[1]))
		extr_nors_z.append(float(vals[2]))
	else:
		print("unclassified val: " + str(val))

dred = (0.5, 0.0, 0.0)
lred = (1.0, 0.5, 0.5)
dblue = (0.0, 0.0, 0.5)
lblue = (0.5, 0.5, 1.0)

white = (1.0, 1.0, 1.0)

print("num entries: " + str(numEntries))

print("extremity point count: " + str(len(extr_pts_y)))
print("extremity norms count: " + str(len(extr_nors_y)))

ax.scatter(extr_pts_x, extr_pts_y, extr_pts_z, c=dred)
ax.scatter(extr_nors_x, extr_nors_y, extr_nors_z, c=lred)

print("sample point count: " + str(len(sample_pts_x)))
print("sample norms count: " + str(len(sample_nors_x)))

ax.scatter(sample_pts_x, sample_pts_y, sample_pts_z, c=dblue)
ax.scatter(sample_nors_x, sample_nors_y, sample_nors_z, c=lblue)

ax.scatter(bounds, bounds, bounds, c=white)

pyplot.show()