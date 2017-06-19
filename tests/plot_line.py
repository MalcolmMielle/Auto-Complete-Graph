import numpy 
import scipy.stats
import matplotlib.pyplot as plt

list1 = [666.658, 665.043, 664.844, 664.776, 664.751, 664.741, 664.737, 664.736, 664.735, 664.735, 668.761, 670.966, 671.644, 671.808, 671.858, 671.879, 671.891, 671.898, 671.903, 671.906, 671.909, 671.911, 671.912, 671.913, 671.914, 671.914, 671.915, 671.915, 671.915, 671.915,]
list2 = list()

print(list1)

for i in range(0, 30):
	list2.append(i)

## PRINTING

#font = {'family', 'normal',
        #'weight', 'bold',
        #'size'  , 22}

#plt.rc('font', **font)
plt.rcParams.update({'font.size': 30})
plt.figure(1)

plt.plot(list2, list1, linestyle='-', marker='o')


plt.xlabel('percentage')
plt.axis([0, max(list2), min(list1) - 10, max(list1) + 10])


plt.show()
