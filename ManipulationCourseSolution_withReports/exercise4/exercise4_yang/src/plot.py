import matplotlib.pyplot as plt
import csv

t = []
x = []
y = []
z = []
fx = []
fy = []
fz = []
mx = []
my = []
mz = []

with open('/tmp/exercise4.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        t.append(float(row[0]))
        x.append(float(row[1]))
        y.append(float(row[2]))
        z.append(float(row[3]))
        fx.append(float(row[4]))
        fy.append(float(row[5]))
        fz.append(float(row[6]))
        mx.append(float(row[7]))
        my.append(float(row[8]))
        mz.append(float(row[9]))

ax1 = plt.subplot(321)
plt.plot(t, fx, label='fx')
plt.legend()
plt.ylim(-2,2)

plt.subplot(323, sharex=ax1)
plt.plot(t, fy, label='fy')
plt.legend()
plt.ylim(-2,2)

plt.subplot(325, sharex=ax1)
plt.plot(t, fz, label='fz')
plt.legend()
plt.ylim(-3,1)

plt.subplot(322, sharex=ax1)
plt.plot(t, x, label='x')
plt.legend()


plt.subplot(324, sharex=ax1)
plt.plot(t, y, label='y')
plt.legend()


plt.subplot(326, sharex=ax1)
plt.plot(t, z, label='z')
plt.legend()

plt.xlim(4,6)
plt.show()