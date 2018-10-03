#!/usr/bin/python
import sys
import numpy as np


def readP3D(fname):
    if type(fname) is str:
        try:
            f=open(fname,'r')
        except:
            print 'Error opening file '+fname
            return None
    elif type(fname) is file:
        f = fname
    try:
        P=np.array(f.read().split()).astype(np.float32).reshape(-1,3)
    except:
        print 'Error reading points from '+fname
        P = None
    if type(fname) is str:
        f.close()
    return P

def saveP3D(fname, P):
    if type(fname) is str:
        try:
            f = open(fname, 'w')
        except:
            print 'Could not open file '+fname+' for writing\n'
            return
    elif type(fname) is file:
        f = fname
    else:
        return
    try:
        np.savetxt(f,P);
    except:
        print 'Error writing data in Magrit.IDeaS.saveP3D\n'

    if type(fname) is str:
        f.close()

#smoothing a 3D curve by a Gaussian RBF
if __name__ == '__main__':
    P=readP3D(sys.argv[1])
    dist=float(sys.argv[2])
    # generate curvilinear abscissae
    n=[0]
    n=np.hstack((n,np.linalg.norm(P[1:,:] - P[:-1,:],axis=1).cumsum()))
    # divide by dist and take the integer part: each time it
    # changes gives the segment to locate the marker to
    # insert
    ni=np.floor(n/dist)
    nd=[0]
    nd=np.hstack((nd,ni[1:]-ni[:-1]))
    idx,=np.where(nd>=1)

    # compute the marker locations by linear interpolation
    a=np.tile(n[idx]-np.floor(n[idx]),(3,1)).T
    Q=P[0]
    Q=np.vstack((Q,a*P[idx-1,:]+(1-a)*P[idx,:]))
    
    saveP3D(sys.argv[3],Q)
