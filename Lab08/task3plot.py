import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyACM0', 115200)  # Adjust the port and baud rate as needed
plt.ion()

# === Plot Styles to Rotate Through ===
plot_styles = ['r.-', 'b.-', 'g.-', 'm.-', 'c.-', 'y.-', 'k.-']

# === Data Storage ===
NUM_SAMPLES = 100
time_ms = []
cnt = 0
all_signals = []  # List of lists. all_signals[0] = i values, all_signals[1] = j values, etc.
signals = ["Temperature", "X", "Y", "Z"]

# === Plotting Function ===
def makeFig(title, xLabel, yLabel, yLimit, *args):
    plt.clf()
    plt.title(title)
    plt.grid(True)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    plt.ylim(yLimit[0], yLimit[1])  

    for arg in args:
        plt.plot(time_ms, arg[0], arg[1], label=arg[2])

    plt.legend(loc='upper left')

    # === Main Loop ===
while True:
    try:
        if sinWaveData.in_waiting == 0:
            continue  # Skip if no data waiting

        line = sinWaveData.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        values = line.split(',')

        # Ensure we received the exact expected number of values (Temperature, X, Y, Z = 4)
        if len(values) != 4:
            continue  # Drop malformed packets instead of crashing or resetting

        # Convert to float safely
        float_values = []
        for val in values:
            float_values.append(float(val))

        # Initialize signal lists on the first valid read
        if not all_signals or len(all_signals) != len(float_values):
            all_signals = [[] for _ in range(len(float_values))]

        # Append each value to its corresponding signal list
        for i, val in enumerate(float_values):
            all_signals[i].append(val)

        time_ms.append(cnt)
        cnt += 1

        # Build args for plotting
        args = []
        for i, signal in enumerate(all_signals):
            style = plot_styles[i % len(plot_styles)]
            label = signals[i]
            args.append((signal, style, label))

        title = "Gyroscope Plot"
        drawnow(lambda: makeFig(title, "Time (ms)", "Value", [-50, 50], *args))
        plt.pause(0.0001)

        # Keep only last NUM_SAMPLES samples
        if len(time_ms) > NUM_SAMPLES:
            time_ms.pop(0)
            for signal in all_signals:
                signal.pop(0)

    except Exception as e:
        print("Runtime Error:", e)