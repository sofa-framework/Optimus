import numpy as np
import sys

f1 = sys.argv[1]
f2 = sys.argv[2]

a1 = np.loadtxt(f1)
a2 = np.loadtxt(f1)

df = np.subtract(a1, a2)

dfn = np.linalg.norm(df, axis=1)

print "  Mean difference norm: ", np.mean(dfn)
print "  Max difference norm: ", np.max(dfn)
