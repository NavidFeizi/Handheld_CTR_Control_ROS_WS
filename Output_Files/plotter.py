import numpy as np
import matplotlib.pyplot as plt
import os
# import seaborn as sns



def remove_outliers(data, threshold=1.5):
    q25, q75 = np.percentile(data, 25, axis=0), np.percentile(data, 75, axis=0)
    iqr = q75 - q25
    cut_off = iqr * threshold
    lower, upper = q25 - cut_off, q75 + cut_off
    outliers = np.any((data < lower) | (data > upper), axis=1)
    filtered_data = data[~outliers]
    return filtered_data, outliers

# Read data from the .dat file
folderName = "2024-08-31_16-00-12";
robot_data = np.genfromtxt("Output_Files/" + folderName + "/Robot.csv", delimiter=",", dtype=float)
FT_data = np.genfromtxt("Output_Files/" + folderName + "/FTsensor.csv", delimiter=",", dtype=float)
EMT_data = np.genfromtxt("Output_Files/" + folderName + "/EMtracker.csv", delimiter=",", dtype=float)

# Separate the columns into x, y, and z
t = robot_data[:,0];

q = robot_data[:,1:7];
q_dot = robot_data[:,7:13];
pos = robot_data[:,13:19];
pos_abs = robot_data[:,19:25];
velocity = robot_data[:,25:31];
current = robot_data[:,31:37];
Force = FT_data[:, 1:4] 
Pos = EMT_data[:, 1:4] 


# Create a 3D scatter plot
column = 2;
fig, ax = plt.subplots(4, 1, figsize=(14, 7))

ax[0].plot(t, current[:,column]*1e3, c="b", label="Current", linewidth=1)  # Scatter plot with red circles
# ax[0].set_xlabel(r"$\mathrm{Time}$ [s]")
ax[0].set_ylabel(r"$\mathrm{current}$ [mA]")
ax[0].minorticks_on()
ax[0].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
ax[0].legend()

ax[1].plot(t, q[:, column]*1e3, c="black", label="Target", linewidth=1.5)  # Scatter plot with red circles
ax[1].plot(t, pos[:, column]*1e3, c="red", label="Position", linewidth=1.5)  # Scatter plot with red circles
ax[1].set_ylabel(r"$\mathrm{joint}$ [mm]")
ax[1].minorticks_on()
ax[1].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
ax2 = ax[1].twinx()
ax2.plot(t, velocity[:, 3] * 1e3, c="blue", label="Velocity", linewidth=1)  # Position plot
ax2.plot(t, q_dot[:, 3] * 1e3, c="cyan", label="q_dot", linewidth=1)  # Position plot
ax2.set_ylabel(r"$\mathrm{joint}$ [mm/s]")
lines, labels = ax[1].get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines + lines2, labels + labels2, loc="upper right")

ax[2].plot(t, Force[:,2], c="b", label="Force", linewidth=1.5)  # Scatter plot with red circles
# ax[2].set_xlabel(r"$\mathrm{Time}$ [s]")
ax[2].set_ylabel(r"$\mathrm{force}$ [N]")
ax[2].minorticks_on()
ax[2].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
ax[2].legend()

ax[3].plot(t, Pos[:,0]*1E3, c="r", label="X", linewidth=1.5)  # Scatter plot with red circles
ax[3].plot(t, Pos[:,1]*1E3, c="b", label="Y", linewidth=1.5)  # Scatter plot with red circles
ax[3].plot(t, Pos[:,2]*1E3, c="g", label="Z", linewidth=1.5)  # Scatter plot with red circles
ax[3].set_xlabel(r"$\mathrm{Time}$ [s]")
ax[3].set_ylabel(r"$\mathrm{tip}$ [mm]")
ax[3].minorticks_on()
ax[3].grid(which="both", color="lightgray", linestyle="--", linewidth=0.5)
ax[3].legend()

plt.subplots_adjust(hspace=0.3)  # Set the spacing between subplots
plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), folderName, 'plot.pdf'), format='pdf')
plt.show(block=True)


# # Error Distribution Plots
# plt.figure(figsize=(12, 6))

# # Histogram
# plt.subplot(1, 3, 1)
# plt.hist(error_x, bins=30, color="blue", alpha=0.7, label="Error X")
# plt.hist(error_y, bins=30, color="red", alpha=0.7, label="Error Y")
# plt.hist(error_z, bins=30, color="green", alpha=0.7, label="Error Z")
# plt.xlabel("Error [mm]")
# plt.ylabel("Frequency")
# plt.legend()
# plt.title("Error Histograms")

# # Boxplot
# plt.subplot(1, 3, 2)
# sns.boxplot(data=[error_x, error_y, error_z], palette=["blue", "red", "green"])
# plt.xticks([0, 1, 2], ["Error X", "Error Y", "Error Z"])
# plt.title("Error Boxplot")

# # Violin plot
# plt.subplot(1, 3, 3)
# sns.violinplot(data=[error_x, error_y, error_z], palette=["blue", "red", "green"])
# plt.xticks([0, 1, 2], ["Error X", "Error Y", "Error Z"])
# plt.title("Error Violin Plot")

# plt.tight_layout()
# plt.show()

# except FileNotFoundError:
#     print("The file 'EM_Trajectory.dat' was not found.")
# except ValueError:
#     print("Error: Unable to parse the data. Please check the format in the file.")
# except Exception as e:
#     print(f"An error occurred: {e}")
