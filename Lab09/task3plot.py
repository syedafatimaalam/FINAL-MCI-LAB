import serial
import matplotlib.pyplot as plt
from drawnow import drawnow

# === Setup Serial ===
PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

sinWaveData = serial.Serial(PORT, BAUD_RATE, timeout=1)

plt.ion()

# === Plot Styles ===
plot_styles = ['r.-', 'b.-', 'g.-', 'm.-', 'c.-']

# === Data Storage ===
NUM_SAMPLES = 100

time_ms = []
cnt = 0

all_signals = [[], [], [], [], []]

signals = ["Accel X", "Accel Y", "Accel Z", "Roll", "Pitch"]

# === Plotting Function ===
def makeFig():
    plt.clf()

    # Acceleration graph
    plt.subplot(2, 1, 1)
    plt.title("LSM303DLHC Accelerometer - Task 2")
    plt.grid(True)
    plt.xlabel("Sample Number")
    plt.ylabel("Acceleration (g)")

    for i in range(3):
        plt.plot(
            time_ms,
            all_signals[i],
            plot_styles[i],
            label=signals[i]
        )

    plt.legend(loc='upper left')

    # Roll and pitch graph
    plt.subplot(2, 1, 2)
    plt.title("Roll and Pitch Angles")
    plt.grid(True)
    plt.xlabel("Sample Number")
    plt.ylabel("Angle (degrees)")

    for i in range(3, 5):
        plt.plot(
            time_ms,
            all_signals[i],
            plot_styles[i],
            label=signals[i]
        )

    plt.legend(loc='upper left')
    plt.tight_layout()


# === Main Loop ===
try:
    while True:
        try:
            line = sinWaveData.readline().decode(
                'utf-8', errors='ignore'
            ).strip()

            if not line:
                continue

            values = line.split(',')

            # Task 2 must contain exactly five values
            if len(values) != 5:
                print("Skipping:", line)
                continue

            # Convert strings to floating-point numbers
            values = [float(val) for val in values]

            # Store readings
            for i, val in enumerate(values):
                all_signals[i].append(val)

            time_ms.append(cnt)
            cnt += 1

            # Keep only the latest 100 samples
            if len(time_ms) > NUM_SAMPLES:
                time_ms.pop(0)

                for signal in all_signals:
                    signal.pop(0)

            # Update graph
            drawnow(makeFig)
            plt.pause(0.001)

        except ValueError:
            print("Invalid data:", line)

except KeyboardInterrupt:
    print("\nPlotting stopped.")

finally:
    sinWaveData.close()
    plt.close('all')