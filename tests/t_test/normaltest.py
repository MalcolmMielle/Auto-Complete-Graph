import numpy 
import scipy.stats
import matplotlib.pyplot as plt

list1 = [67, 74, 75, 69, 65, 65, 66, 64, 69, 73, 70, 65, 68, 61.5]
list2 = [96, 86, 91, 91, 77, 93, 86, 81, 71, 94, 74, 93, 85, 64]
list3 = [71, 71]
list4 = [78, 78]

def allStat(list_in, list_in2):
	print("T test")
	res = scipy.stats.ttest_ind(list_in, list_in2, equal_var = False) #<- since False, no need for homogeneity of variance for we use Welsch t-test
	print(res.statistic)
	print(res.pvalue)
	if res.pvalue <= 0.05:
		print("COOL on a un interval de confiance de moins de 5% blabla. Statiquemtne t les deux listes sont differentes dans un interval de 5% (5% derreur accepté :P)")


def mean(list):
	sum = 0
	for element in list:
		sum = sum + element
	sum = sum / len(list)
	return sum

def variance(list, mean):
	sum_el = 0 
	for element in list:
		temp_el = element - mean
		temp_el = temp_el * temp_el
		sum_el = sum_el + temp_el
	sum_el = sum_el / (len(list) - 1)
	return sum_el
	
def sd(variance):
	standd = numpy.sqrt(variance)
	return standd


def z_score(val, normal):
	return (val - numpy.mean(normal)) / numpy.std(normal, ddof = 1)
	
	
def sevent3(z_score_max, z_score_min, sd_v):
	if(z_score_max >= 3*sd_v or z_score_max <= -3*sd_v):
		print("NO max")
		return False
	elif(z_score_min >= 3*sd_v or z_score_min <= -3*sd_v):
		print("NO min")
		return False
	else:
		print("YES")
		return True
	
	
	
print("Mean")
print(mean(list1))
print("Variance")
print(variance(list1, mean(list1)))
print("SD")
print(sd(variance(list1, mean(list1))))
	
#Calculating the normal
	
mean_v = mean(list1)
sd_v = sd(variance(list1, mean_v))

normal = numpy.random.normal(mean_v, sd_v, 1000)

print("variance")
print(numpy.std(normal, ddof = 1))

print("max min")
print(max(list1))
print(min(list1))

mean_v_2 = mean(list2)
sd_v_2 = sd(variance(list2, mean_v_2))

normal_2 = numpy.random.normal(mean_v_2, sd_v_2, 1000)

# ASSESSING THE NORMALITY
print("Is it normal list 1 ? ")

z_score_max = z_score(max(list1), normal)
z_score_min = z_score(min(list1), normal)
print(z_score_max)
print(z_score_min)
is_normal = sevent3(z_score_max, z_score_min, sd_v)

print("Is it normal list 2 ? ")
z_score_max_2 = z_score(max(list2), normal)
z_score_min_2 = z_score(min(list2), normal)
print(z_score_max_2)
print(z_score_min_2)
is_normal_2 = sevent3(z_score_max_2, z_score_min_2, sd_v_2)

print("normality of list 1: ", is_normal, " and list 2: ", is_normal_2)
# ASSESSING HOMOGENEITY OF VARIANCE

# T test

print("All stat list1 and 2")
allStat(list1, list2)

print("All stat list1 and 70%")
allStat(list1, list3)

print("All stat list2 and 80%")
allStat(list2, list4)



## PRINTING

#font = {'family' : 'normal',
        #'weight' : 'bold',
        #'size'   : 22}

#plt.rc('font', **font)
plt.rcParams.update({'font.size': 50})

plt.figure(1)
count, bins, ignored = plt.hist(normal, 30, normed=True)
#plt.clf()
plt.figure(2)
plt.axvspan(70, 80, alpha=0.5, color='red')
#plt.plot(bins, 1/(sd_v * numpy.sqrt(2 * numpy.pi)) *
               #numpy.exp( - (bins - mean_v)**2 / (2 * sd_v**2) ),
         #linewidth=4, color='r')
#plt.clf()

#plt.axvline(mean_v - (sd_v*3), color='r')
#plt.axvline(mean_v + (sd_v*3), color='r')

max_c = max(count)

bottom1 = list()
size1 = list()
for el in list1:
	bottom1.append(0.4)
	size1.append(700)


#bottom1[4] = bottom1[4]*2
#bottom1[5] = bottom1[5]*2

plt.scatter(list1, bottom1, size1, 'r')
#plt.show()

#plt.clf()

plt.figure(1)
count_2, bins_2, ignored_2 = plt.hist(normal_2, 30, normed=True)
#plt.clf()
plt.figure(2)
#plt.plot(bins_2, 1/(sd_v_2 * numpy.sqrt(2 * numpy.pi)) *
               #numpy.exp( - (bins_2 - mean_v_2)**2 / (2 * sd_v_2**2) ),
         #linewidth=4, color='g')

#plt.axvline(mean_v_2 - (sd_v_2*3), color='g')
#plt.axvline(mean_v_2 + (sd_v_2*3), color='g')

max_c_2 = max(count_2)

bottom = list()
size = list()
for el in list2:
	bottom.append(0.2)
	size.append(700)

#bottom[2] = bottom[2]*2
#bottom[3] = bottom[3]*2

plt.scatter(list2, bottom, size, 'g')

#Draw the grey zone

plt.xlabel('percentage')
plt.axis([60, 100, 0, 0.5])


plt.show()
