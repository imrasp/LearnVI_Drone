from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
import csv

x,y,z = [],[],[]
nedx,nedy,nedz = [],[],[]
gpsx,gpsy,gpsz = [],[],[]
gpslat,gpslon,gpsalt = [],[],[]
oldgpsx,oldgpsy,oldgpsz = [],[],[]
scaledx, scaledy, scaledz = [],[],[]
with open('visionEstimate2IMULog.csv', 'rb') as csvfile:
	freader = csv.DictReader(csvfile, delimiter=',')
	for row in freader:
		x.append(float(row['x']))
		y.append(float(row['y']))
		z.append(float(row['z']))
		scaledx.append(float(row['scaled_x']))
		scaledy.append(float(row['scaled_y']))
		scaledz.append(float(row['scaled_z']))

with open('gpsdata.csv', 'rb') as csvfile:
	freader = csv.DictReader(csvfile, delimiter=',')
	for row in freader:
		nedx.append(float(row['x']))
		nedy.append(float(row['y']))
		nedz.append(float(row['z']))
		gpsx.append(float(row['gpsx']))
		gpsy.append(float(row['gpsy']))
		gpsz.append(float(row['gpsz']))
		gpslat.append(float(row['lat']))
		gpslon.append(float(row['lon']))
		gpsalt.append(float(row['alt']))

with open('gpsdata_original_coonversion.csv', 'rb') as csvfile:
	freader = csv.DictReader(csvfile, delimiter=',')
	for row in freader:
		oldgpsx.append(float(row['gpsx']))
		oldgpsy.append(float(row['gpsy']))
		oldgpsz.append(float(row['gpsz']))
print scaledx, scaledy
fig = plt.figure()
fig.suptitle("Position Comparision", fontsize=16)

ax = fig.add_subplot(231, projection='3d')
ax.set_title('vision x y z')
ax.plot_wireframe(x, y, z, color = 'r')

ay = fig.add_subplot(232, projection='3d')
ay.set_title('vision scaledx scaledy scaledz')
ay.plot_wireframe(scaledx, scaledy, scaledz, color = 'r')

#ay = fig.add_subplot(232, projection='3d')
#ay.set_title('converted GPS x y z')
#ay.plot_wireframe(gpsx, gpsy, gpsz, color = 'lime')
#ay.plot_wireframe( oldgpsy, oldgpsz, oldgpsx,color = 'purple')

ai = fig.add_subplot(233, projection='3d')
ai.set_title('NED x y z')
ai.plot_wireframe(nedx, nedy, nedz, color = 'deepskyblue')

az = fig.add_subplot(234, projection='3d')
az.set_title('GPS lat lon alt')
az.plot_wireframe(gpslat, gpslon, gpsalt, color = 'salmon')

#aj = fig.add_subplot(235, projection='3d')
#aj.set_title('old converted GPS x y z')
#aj.plot_wireframe(oldgpsx, oldgpsy, oldgpsz, color = 'purple')

plt.show() 
