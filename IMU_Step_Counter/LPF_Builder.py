import numpy as np
from scipy import signal

freq_Hz = 5       # Frequency of overall IMU signal (Currently chosen arbitrarily)

#-----------------------------Continuous Transfer Function-------------------------------
wc = 2*np.pi*freq_Hz                # Cutoff Frequency
n = 2                               # Filter Order
                # Higher order: Better Filter but More Complexity (Longer Delay)
                        
# Compute Filter Coefficients
butter = np.zeros(n+1)
gamma = np.pi/(2*n)
butter[0] = 1
for i in range(n):
    temp = np.cos(i*gamma)/np.sin((i+1)*gamma)
    butter[i+1] = temp*butter[i]

denom = np.zeros(n+1)
for i in range(n+1):
    denom[n-i] = butter[i]/(wc**i)

coeffs_num = [1]
coeffs_den = denom
lowPass = signal.TransferFunction(coeffs_num, coeffs_den)
#----------------------------------------------------------------------------------------

# Discrete Transfer Function
fs = 1/1000        # Arbitrarily set sampling freq to 1kHz
lowPass_dis = lowPass.to_discrete(fs, method='gbt', alpha=0.5)

# Difference Equation Coefficients
b = lowPass_dis.num
a = lowPass_dis.den[1:]

print(b)
print(a)