import os
import matplotlib.pyplot as plt
import numpy as np

os.chdir(homerinGithublinked-list)

def stat(data)
    data = np.array(data)
    ret = np.zeros(data.shape[1])

    for i in range(data.shape[1])
        tmp = data[,i]
        mean, std = tmp.mean(), tmp.std()
        # identify outliers 2 std = 95%
        cut_off = std  2
        lower, upper = mean - cut_off, mean + cut_off
        tmp = tmp[tmp  lower]
        tmp = tmp[tmp  upper]
        ret[i] = tmp.mean()

    return ret

old_time_collect = []
new_time_collect = []


for i in range(10)
    os.system(.exec  .plotout.txt)
    out = np.loadtxt(.plotout.txt, dtype = 'float',delimiter=',')
  
    old_time_collect.append(out[,1])
    new_time_collect.append(out[,2])

x = out[,0]
old_time_collect = stat(old_time_collect)
new_time_collect = stat(new_time_collect)


plt.plot()
plt.xlabel('node #')
plt.ylabel('time(ms)')

plt.plot(x, old_time_collect, label = 'swap node')
plt.plot(x, new_time_collect, label = 'swap by value')
plt.legend(loc = 'upper left')
plt.show()
