import numpy 
import scipy.stats
import matplotlib.pyplot as plt

list1 = [0.5, 1, 2, 2, 2, 1, 1, 0.5, 0.5, 0.5]
list2 = [0.5, 1, 2, 2, 2, 1, 1, 0.5, 0.5, 0.5]

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

# ASSESSING THE NORMALITY

z_score_max = z_score(max(list1), normal)
z_score_min = z_score(min(list1), normal)

print(z_score_max)
print(z_score_min)

print("Is it normal ? ")

is_normal = sevent3(z_score_max, z_score_min, sd_v)

## ASSESSING HOMOGENEITY OF VARIANCE

## T test

print("T test")
res = scipy.stats.ttest_ind(list1, list2, False) #<- since False, no need for homogeneity of variance for we use Welsch t-test
print(res.statistic)
print(res.pvalue)

## PRINTING

count, bins, ignored = plt.hist(normal, 30, normed=True)
plt.plot(bins, 1/(sd_v * numpy.sqrt(2 * numpy.pi)) *
               numpy.exp( - (bins - mean_v)**2 / (2 * sd_v**2) ),
         linewidth=2, color='r')
plt.show()
