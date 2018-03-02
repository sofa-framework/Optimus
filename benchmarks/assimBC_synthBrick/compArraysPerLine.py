import numpy as np
import sys

f1 = sys.argv[1]
f2 = sys.argv[2]

a1 = np.loadtxt(f1)
a2 = np.loadtxt(f2)

df = np.subtract(a1, a2)
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
