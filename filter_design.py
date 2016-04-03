import scipy.signal
import numpy
import math

def transformInv(b, a, T, tol = 1E-8):
  (r, p, k) = scipy.signal.residue(b, a, tol = tol)
  k = k - numpy.sum(r)/2
  print "Residue:", k
  k = k * T
  r = r * T
  (db, da) = scipy.signal.invresz(r, numpy.exp(p*T), k, tol = tol)
  db = db.real
  da = da.real
  return (db, da)

def design(fsample, fpass, fstop, gpass, gstop, ftype = 'butter', tol = 1E-8):
  (b, a) = scipy.signal.iirdesign(2*math.pi*fpass, 2*math.pi*fstop, float(gpass), float(gstop), analog = True, ftype = ftype)
  #print "Analog filter:", b, a
  (db, da) = transformInv(b, a, 1.0/fsample, tol = tol)
  return (db, da)



fsamp = 2000
fpass =  450
fstop =  550
gpass = 1
gstop = 40

(b, a) = design(fsamp, fpass, fstop, gpass, gstop, 'ellip')

print "Order:", max([len(b), len(a)])
print "Digital filter:", b, a

(w, H) = scipy.signal.freqz(b, a, 2*math.pi*numpy.arange(400, 650, 50)/fsamp)
print "Freq:", w/(2*math.pi)*fsamp
print "Mag:", 20 * numpy.log10(numpy.abs(H))
