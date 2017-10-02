import scipy.signal
import numpy
import math
import sys

def impinvar(b, a, fs, tol = 1E-8):
    # https://ccrma.stanford.edu/~jos/pasp/Impulse_Invariant_Method.html
    # https://en.wikipedia.org/wiki/Impulse_invariance
    (r, p, k) = scipy.signal.residue(b, a, tol = tol)
    # Correction for consistency
    k = k - (numpy.sum(r)/2).real
    T = 1.0 / fs
    print "Poles:", numpy.exp(p*T)
    print "Residue:", k
    (db, da) = scipy.signal.invresz(r*T, numpy.exp(p*T), k*T, tol = tol)
    db = db.real
    da = da.real
    return (db, da)

def design(fsample, fpass, fstop, gpass, gstop, ftype = 'butter', tol = 1E-8):   
    wpass = 2 * math.pi * fpass
    wstop = 2 * math.pi * fstop
    #(b, a) = scipy.signal.iirdesign(wpass, wstop, gpass, gstop, analog = True, ftype = ftype)
    #(db, da) = impinvar(b, a, fsample, tol = tol)
    (db, da) = scipy.signal.iirdesign(2.0 * fpass / fsample, 2.0 * fstop / fsample, gpass, gstop, analog = False, ftype = ftype)
    return (db, da)

def design_sos(fsample, fpass, fstop, gpass, gstop, ftype = 'butter', tol = 1E-8):   
    fny = fsample / 2.0
    sos = scipy.signal.iirdesign(fpass / fny, fstop / fny, gpass, gstop, 
                analog = False, ftype = ftype, output='sos')
    return sos

def export_sos(sos, stream):
    ''' The coefficients are stored in the array pCoeffs in the following order:
            {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
        where b1x and a1x are the coefficients for the first stage, b2x and a2x are the coefficients for the second stage, and so on.
        
        Each Biquad stage implements a second order filter using the difference equation:
            y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2]
    '''
    coefs = []
    for (b0, b1, b2, a0, a1, a2) in sos:
        coefs += (b0, b1, b2, -a1, -a2)     # need to negate a1 and a2 for CMSIS DSP

    line = ', '.join(["%.7ef" % x for x in coefs])
    stream.write("%s\n" % line)

fsamp = 10000
fpass =  1350
fstop =  1450
gpass =   1.0
gstop =  60.0

(b, a) = design(fsamp, fpass, fstop, gpass, gstop, 'ellip')
sos    = design_sos(fsamp, fpass, fstop, gpass, gstop, 'ellip')

print "Order:", max([len(b), len(a)])
print "Numerator   (b):", b
print "Denominator (a):", a
print "2nd order sections:", sos

(w, H) = scipy.signal.freqz(b, a, 2*math.pi*numpy.arange(fpass - 100, fstop + 100, 50)/fsamp)
print "Freq:", w/(2*math.pi)*fsamp
print "Mag:", 20 * numpy.log10(numpy.abs(H))

(w, H) = scipy.signal.sosfreqz(sos, 2*math.pi*numpy.arange(fpass - 100, fstop + 100, 50)/fsamp)
print "Freq:", w/(2*math.pi)*fsamp
print "Mag:", 20 * numpy.log10(numpy.abs(H))

# 300 Hz highpass 
export_sos(design_sos(fsamp, 300.0, 270.0, gpass, gstop, 'ellip'), sys.stdout)

# 2700 Hz lowpass
export_sos(design_sos(fsamp, 2700.0, 3000.0, gpass, gstop, 'ellip'), sys.stdout)

# 1400 Hz lowpass
export_sos(design_sos(fsamp, 1350.0, 1450.0, gpass, gstop, 'ellip'), sys.stdout)
