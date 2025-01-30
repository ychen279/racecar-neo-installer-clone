import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde

# Generate multimodal data
np.random.seed(42)
data = np.concatenate([
    np.random.normal(0, 1, 300),
    np.random.normal(5, 0.1, 300),
    np.random.normal(10, 1, 300)
])

# Kernel Density Estimation (KDE)
kde = gaussian_kde(data)
x = np.linspace(min(data), max(data), 1000)
density = kde(x)
x = (x-x[0])/(x[-1]-x[0])
density[-1] = density[0]


# Find peaks in KDE
peaks, _ = signal.find_peaks(density)
widths = signal.peak_widths(density,peaks)


# Use FFT to do the same
fft_result = np.fft.fft(density)
frequencies = np.fft.fftfreq(len(x),1/len(x))
sizefft = np.abs(fft_result)
mode1idx = np.argsort(sizefft)[-1]
mode2idx = np.argsort(sizefft)[-2]
mode3idx = np.argsort(sizefft)[-3]
mode4idx = np.argsort(sizefft)[-4]
print(frequencies[mode3idx])
M1 = np.real(fft_result[mode1idx]) * np.cos(2 * np.pi * frequencies[mode1idx] * x) + np.imag(fft_result[mode1idx]) * np.sin(2 * np.pi * frequencies[mode1idx] * x)
M2 = np.real(fft_result[mode2idx]) * np.cos(2 * np.pi * frequencies[mode2idx] * x) + np.imag(fft_result[mode2idx]) * np.sin(2 * np.pi * frequencies[mode2idx] * x)
M3 = np.real(fft_result[mode3idx]) * np.cos(2 * np.pi * frequencies[mode3idx] * x) + np.imag(fft_result[mode3idx]) * np.sin(2 * np.pi * frequencies[mode3idx] * x)
M4 = np.real(fft_result[mode4idx]) * np.cos(2 * np.pi * frequencies[mode4idx] * x) + np.imag(fft_result[mode4idx]) * np.sin(2 * np.pi * frequencies[mode4idx] * x)

print(sizefft)

# Plot
plt.plot(x, density, label="KDE")
plt.scatter(x[peaks], density[peaks], color='red', label="Peaks")
plt.legend()
plt.title("Peak Detection in Multimodal Distribution")
plt.savefig("XXX.jpg")
plt.close("all")


plt.plot(x, M1, label="KDE")
plt.legend()
plt.savefig("Mode1.jpg")
plt.close("all")

plt.plot(x, M2, label="KDE")
plt.legend()
plt.savefig("Mode2.jpg")
plt.close("all")

plt.plot(x, M3, label="KDE")
plt.legend()
plt.savefig("Mode3.jpg")
plt.close("all")

plt.plot(x, M4, label="KDE")
plt.legend()
plt.savefig("Mode4.jpg")
plt.close("all")

plt.plot(x, M1+M2+M3+M4, label="KDE")
plt.legend()
plt.savefig("All.jpg")
plt.close("all")