import numpy as np
import sys

f1 = sys.argv[1]        #first file: benchmark data
f2 = sys.argv[2]		#second file: testing data
nl=int(sys.argv[3])    			#number of lines to check

a1 = np.loadtxt(f1)
a2 = np.loadtxt(f2)

sh1 = a1.shape
sh2 = a2.shape

if sh1[1] != sh2[1]:
	print 'Error: arrays have different number of columns, cannot compare'
	exit()

if sh1[0] < nl:
	print 'Number of rows in first data (',sh1[0],') is smaller than required size ',nl
	exit()

if sh2[0] < nl:
	print 'Number of rows in second data (',sh2[0],') is smaller than required size ',nl	
	exit()

b1 = a1[:nl,:]
b2 = a2[:nl,:]

df = np.subtract(b1, b2)
#dfrel = np.divide(df, a1)

dfn = np.linalg.norm(df, axis=1)
#dfnrel = np.linalg.norm(dfrel, axis=1)

# print "  Mean row-wise norm of difference: abs = ",  np.mean(dfn), " rel = ", np.mean(dfnrel)
# print "  Max  row-wise norm of difference: abs = ", np.max(dfn), " rel = ", np.max(dfnrel)
# print "  Element-wise mean of difference: abs = ", np.mean(df), " rel = ", np.mean(dfrel)
# print "  Element-wise max  of difference: abs = ", np.max(df), " rel = ", np.max(dfrel)

print "  Mean row-wise norm of difference: ",  np.mean(dfn)
print "  Max  row-wise norm of difference: ", np.max(dfn)
print "  Element-wise mean of difference: ", np.mean(df)
print "  Element-wise max  of difference: ", np.max(df)
