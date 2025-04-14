import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import iirnotch, filtfilt, butter
import os
import datetime

# ========== paprameters ==========
FS = 200  # sample frequency
WINDOW_TIME = 7  # window time
WINDOW_SIZE = FS * WINDOW_TIME  # window size


timestamp = datetime.datetime.now().strftime("%Y%m%d")
FILENAME = f"EEG_data_{timestamp}.txt"
#FILENAME = f"EEG_data_20250324_190549.txt"

# ========== buffer ==========
time_axis = np.linspace(-WINDOW_TIME, 0, WINDOW_SIZE)
data = np.zeros(WINDOW_SIZE)  
filtered_data = np.zeros(WINDOW_SIZE)

# ========== reading ==========
def read_latest_data():
    
    global data
    if not os.path.exists(FILENAME):
        print(f"file {FILENAME} not exist，waiting for reading...")
        return
    
    try:
        with open(FILENAME, "r") as f:
            lines = f.readlines()[-WINDOW_SIZE:]  
        
        new_data = np.array([float(line.strip()) for line in lines if line.strip()])
        
        new_data[new_data <= 0.1] = np.mean(new_data)
        
        if len(new_data) < WINDOW_SIZE:
            new_data = np.pad(new_data, (WINDOW_SIZE - len(new_data), 0), mode='constant')
        
        data[:] = new_data  #  update to buffer
    
    except Exception as e:
        print(f"Reading Error: {e}")

# ========== Notch filter ==========
def notch_filter(signal, NOTCH_FREQ=50, Q_FACTOR=20):
    b, a = iirnotch(NOTCH_FREQ / (FS / 2), Q=Q_FACTOR)
    if len(signal) < 10:
        return signal
    res = filtfilt(b, a, signal)
    return lowpass_filter(res, FS) #w with a 70 Hz lowpass filter
    

def lowpass_filter(signal, fs=250, cutoff=70, order=4):
    if len(signal) < 10:  
        return signal
    
    nyquist = 0.5 * fs  
    normal_cutoff = cutoff / nyquist  
    b, a = butter(order, normal_cutoff, btype='low', analog=False)  
    return filtfilt(b, a, signal)  

# ========== drawing ==========
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

# raw
line_raw, = ax1.plot(time_axis, data, label="Raw EEG Signal")
ax1.set_ylim(0, 3.4)
ax1.set_title("EEG Raw Signal")
ax1.set_ylabel("Voltage (V)")
ax1.legend(loc="upper right" )

# after
line_filtered, = ax2.plot(time_axis, filtered_data, label="Filtered EEG (50Hz Notch)", color='red')
ax2.set_ylim(0, 3.4)
ax2.set_title("EEG Signal (50Hz Notch Filtered)")
ax2.set_ylabel("Voltage (V)")
ax2.legend(loc="upper right" )

# FFT 
fft_freqs = np.fft.rfftfreq(WINDOW_SIZE, d=1/FS)
fft_magnitudes = np.zeros_like(fft_freqs)
line_fft, = ax3.plot(fft_freqs, fft_magnitudes, label="FFT Spectrum", color='green')
ax3.set_xlim(0, 125)  
ax3.set_ylim(0, 1)
ax3.set_title("EEG Frequency Spectrum (FFT)")
ax3.set_xlabel("Frequency (Hz)")
ax3.set_ylabel("Magnitude")
ax3.set_xticks(np.arange(0, 126, 5))
ax3.legend(loc="upper right" )

# ========== Update ==========
def update_raw(frame):
    read_latest_data()
    line_raw.set_ydata(data)
    return line_raw,

def update_filtered(frame):
    global filtered_data
    filtered_data = notch_filter(data, 54, 10)
    
    line_filtered.set_ydata(filtered_data)
    return line_filtered,

def update_fft(frame):
    global fft_magnitudes
    fft_values = np.abs(np.fft.rfft(filtered_data - np.mean(filtered_data)))
    #fft_values = np.abs(np.fft.rfft(data - np.mean(data)))
    #fft_values = np.abs(np.fft.rfft(data - np.mean(data)))
    fft_magnitudes = fft_values / np.max(fft_values) if np.max(fft_values) != 0 else fft_values
    line_fft.set_ydata(fft_magnitudes)
    return line_fft,

# ========== animation ==========
ani_raw = FuncAnimation(fig, update_raw, interval=500)  # 500ms for raw
ani_filtered = FuncAnimation(fig, update_filtered, interval=1000)  
ani_fft = FuncAnimation(fig, update_fft, interval=1000)  # 1s for FFT

plt.show()
