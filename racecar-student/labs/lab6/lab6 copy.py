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


# Find peaks in KDE
peaks, _ = signal.find_peaks(density)
widths = signal.peak_widths(density,peaks)

print(peaks)
print(widths[0])

# Plot
plt.plot(x, density, label="KDE")
plt.scatter(x[peaks], density[peaks], color='red', label="Peaks")
plt.legend()
plt.title("Peak Detection in Multimodal Distribution")
plt.savefig("XXX.jpg")
