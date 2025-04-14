import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from scipy.signal import butter, filtfilt, iirnotch
import keyboard  # 

# ========== 1. Parameters ==========
PORT = "COM10"  # use your serial port 
BAUD_RATE = 115200
FS = 100  
WINDOW_TIME = 5  
WINDOW_SIZE = FS * WINDOW_TIME  
FFT_UPDATE_INTERVAL = 1000  

# 
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=0.01)  

    print(f"Connected to {PORT}")
except Exception as e:
    print(f"Failed to connect to {PORT}: {e}")
    exit()

# ========== 2. notch_filter ==========
NOTCH_FREQ = 50.0  
Q_FACTOR = 20.0  
b, a = iirnotch(w0=NOTCH_FREQ / (FS / 2), Q=Q_FACTOR)

def notch_filter(signal):
    if len(signal) < 10:  
        return signal
    return filtfilt(b, a, signal)

# ========== 3. buffer ==========
time_axis = np.linspace(-WINDOW_TIME, 0, WINDOW_SIZE) 
data = np.zeros(WINDOW_SIZE)  
filtered_data = np.zeros(WINDOW_SIZE)  
filtered_buffer = []  
recording = False  
recorded_data = []  

# ========== 4. recording ==========
def toggle_recording(e):
    """Press R to record or stop"""
    global recording, recorded_data
    if e.name == 'r':
        recording = not recording
        if not recording:
            with open("EEG_data.txt", "w") as f:
                for value in recorded_data:
                    f.write(f"{value}\n")
            print("Recording stopped, data saved.")
            recorded_data = []
        else:
            print("Recording started...")

keyboard.on_press(toggle_recording)

# ========== 5. drawing ==========
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

line_raw, = ax1.plot(time_axis, data, label="Raw EEG Signal")
ax1.set_ylim(0, 3.3)
ax1.set_title("EEG Raw Signal")
ax1.set_ylabel("Voltage (V)")
ax1.legend()

line_filtered, = ax2.plot(time_axis, filtered_data, label="Filtered EEG (50Hz Notch)", color='red')
ax2.set_ylim(0, 3.3)
ax2.set_title("EEG Signal (50Hz Notch Filtered)")
ax2.set_ylabel("Voltage (V)")
ax2.legend()

# FFT 
fft_freqs = np.fft.rfftfreq(WINDOW_SIZE, d=1/FS) 
fft_magnitudes = np.zeros_like(fft_freqs)
line_fft, = ax3.plot(fft_freqs, fft_magnitudes, label="FFT Spectrum", color='green')
ax3.set_xlim(0, 100) 
ax3.set_ylim(0, 1)
ax3.set_title("EEG Frequency Spectrum (FFT)")
ax3.set_xlabel("Frequency (Hz)")
ax3.set_ylabel("Magnitude")
ax3.set_xticks(list(range(0, 100, 10)))
ax3.legend()

# ========== 6. update ==========
def update_raw(frame):
    global data, recorded_data, filtered_buffer
    aind = 0
    try:
        while ser.in_waiting:
        #if ser.in_waiting > 0:
            raw_data = ser.readline().decode("utf-8").strip()
            voltage = float(raw_data)
            #if voltage >= 3.3 or voltage <= 0.5: # Power Jump
            #    voltage = np.mean(data)

            # 
            data[:-1] = data[1:]
            data[-1] = voltage  # 
            #aind += 1

            if recording:
                recorded_data.append(voltage)
                
            #filtered_buffer.append(voltage)
            #if len(filtered_buffer) > WINDOW_SIZE:
            #    filtered_buffer.pop(0)  #
    except ValueError:
        print(f"Vlaue Error")
    except Exception as e:
        print(f"Error reading data: {e}")

    line_raw.set_ydata(data)
    #print(aind)
    return line_raw,

# ========== 7. Uodate ==========
def update_filtered(frame):
    global filtered_data

    if len(data) >= WINDOW_SIZE:
        filtered_data = notch_filter(data)  

    line_filtered.set_ydata(filtered_data)
    return line_filtered,


# ========== 8. Update FFT ==========
def update_fft(frame):
    global fft_magnitudes
    fft_values = np.abs(np.fft.rfft(filtered_data-np.mean(filtered_data)))  
    fft_magnitudes = fft_values / np.max(fft_values) if np.max(fft_values) != 0 else fft_values  
    
    line_fft.set_ydata(fft_magnitudes)
    return line_fft,

# ========== 9. animation ==========
ani_raw = FuncAnimation(fig, update_raw, interval=100)  
ani_filtered = FuncAnimation(fig, update_filtered, interval=1000)  
ani_fft = FuncAnimation(fig, update_fft, interval=FFT_UPDATE_INTERVAL)  # 1s update animation

plt.show()

# close BT
ser.close()
