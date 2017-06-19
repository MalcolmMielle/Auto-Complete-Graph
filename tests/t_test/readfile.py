import os
import matplotlib.pyplot as plt


def plot_line(list1):
	list2 = list()
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



chi_mean = list()
flag = False
count_files = 0
for file in os.listdir("./Chi"):
	if file.endswith(".txt"):
		count_files = count_files + 1
		with open(os.path.dirname(__file__)+"Chi/"+file,'r') as f:
			count = 0
			for line in f:
				for word in line.split():
					if word != ':':
						if(flag == False):
							chi_mean.append(float(word))
						else:
							chi_mean[count] = chi_mean[count] + float(word)
							count = count + 1
			flag = True
chi_mean[:] = [x / count_files for x in chi_mean]
print(chi_mean)

plot_line(chi_mean)
