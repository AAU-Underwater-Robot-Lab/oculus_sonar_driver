import sqlite3
import numpy as np
import time
import matplotlib as mpl
mpl.use('tkagg')
import matplotlib.pyplot as plt

con = sqlite3.connect("/home/chrimai/SONAR/20220923_152558.oculus")

cursor = con.cursor();

cursor.execute("SELECT * FROM data")

records = cursor.fetchmany(100);
nbeams = 256;
lenr = int(records[1][4]/nbeams);

print(lenr)
print(nbeams)
print(cursor.description)

used_theta = np.linspace(np.radians(-40/2), np.radians(40/2), nbeams) # 512 beams from -65 to 65 degrees
used_rad = np.linspace(0, 10, lenr)

theta,rad = np.meshgrid(used_theta, used_rad) #rectangular plot of polar data
X = theta
Y = rad

fig = plt.figure()
ax = fig.add_subplot(111, polar='True')


for row in records:
    array = np.fromstring(row[5], 'uint8');
    array = np.reshape(array, (lenr, nbeams));
    #print(array)
    ax.pcolormesh(X, Y, array, shading='auto') #X,Y & data2D must all be same dimensions
    plt.pause(0.001)

