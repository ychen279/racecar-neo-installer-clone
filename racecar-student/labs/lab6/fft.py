import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft, fftfreq

# Generate a sample periodic signal
N = 500  # Number of samples
T = 1.0 / 100.0  # Sampling interval (100 Hz sampling)
x = np.linspace(0.0, N*T, N, endpoint=False)
signal = np.sin(2.0*np.pi*10.0*x) + 0.5*np.sin(2.0*np.pi*25.0*x) + 0.3*np.sin(2.0*np.pi*40.0*x)

# Compute FFT
yf = fft(signal)
xf = fftfreq(N, T)  # Frequency bins

# Find the indices of the top 3 dominant frequencies (excluding DC component)
indices = np.argsort(np.abs(yf[:N//2]))[::-1]  # Sort by magnitude (highest first)
top_modes = indices[:3]  # Select top 3 frequency indices

# Function to reconstruct a single mode
def reconstruct_mode(yf, mode_index):
    yf_mode = np.zeros_like(yf, dtype=complex)
    yf_mode[mode_index] = yf[mode_index]  # Keep only the positive frequency
    yf_mode[-mode_index] = yf[-mode_index]  # Keep symmetric negative frequency
    return np.real(ifft(yf_mode))

# Plot original signal
plt.figure(figsize=(12, 6))
plt.plot(x, signal, label="Original Signal", color="black", alpha=0.7)

# Plot first 3 FFT wave shapes
for i, mode in enumerate(top_modes):
    reconstructed_wave = reconstruct_mode(yf, mode)
    plt.plot(x, reconstructed_wave, label=f"Mode {i+1}: {np.abs(xf[mode]):.1f} Hz")

plt.title("First 3 FFT Modes of the Signal")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.legend()
plt.grid()
plt.savefig("XXX.jpg")
