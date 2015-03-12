import numpy as np 
import matplotlib.pyplot as plt 

filename = "RC1"
with open(filename) as f :
	data = f.read()

rows = data.split('\n')
rows = [ row.split() for row in rows ]
x_arr = []
y_arr = []
for row in rows:
	if row:
		x = float(row[0])
		y = float(row[1])
		x_arr.append(x)
		y_arr.append(y)


fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.set_title(filename)
ax1.set_xlabel('angle in rad')
ax1.set_ylabel('depth')
ax1.plot(x_arr,y_arr,c='r',label='flow')

leg = ax1.legend()
plt.show()



