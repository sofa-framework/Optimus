#!/usr/bin/env python
import sys
import numpy as np
import json

# read vect
def readVect(fname):    
    with open(fname,'r') as fp:
        d=fp.read().split()
        if d[0] != 'VECT':
            print 'Assuming 3D points in plain text'
            return np.array(d).astype(np.float32).reshape((-1,3))
        if d[1] != '1':
            print 'Unable to handle multiple polylines.'
            return None
        npts=np.uint32(d[2])
        # d[3] == ncolors
        # d[4] == nvertices in first (and only) polyline
        # d[5] == ncolors in first (and only) polyline
        P=np.array(d[6:6+npts*3]).astype(np.float32).reshape((-1,3))
    return P

def savePolyLine2D(outf,P):
    with open(outf,'w') as f:
        f.write('DrawablePolyLine2D Cath #0000FF '+str(P.shape[0])+'\n')
        for p in P:
            # store the Z (plane index) as label
            f.write('0 '+str(p[0])+' '+str(p[1])+'\n')

def readProj(fname):
    with open(fname,'r') as f:
        return np.array(f.read().split(),dtype=np.float32).reshape((3,4))

# unsafe: nullity not tested for Z component
def projP3D(P3D,M):
    q=np.dot(M,np.vstack((P3D.T,np.ones(P3D.shape[0]))))
    P2D=q[:2,:]/q[2,:] #unsafe
    return P2D.T
    
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print "Syntax: projPoints3D.py <file.vect> <proj.txt> <out.p2d>"
        exit()
        
    # read points
    P=readVect(sys.argv[1])
    # projection
    M=readProj(sys.argv[2])
    #project
    q=projP3D(P,M)
    #save
    savePolyLine2D(sys.argv[3],q)

