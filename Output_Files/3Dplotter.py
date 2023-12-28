import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import quaternion

def quaternion_to_euler(q):
    # Normalize the quaternion to ensure valid conversion
    q = q.normalized()
    
    # Extract individual components for readability
    w, x, y, z = q
    
    # Calculate Euler angles (roll, pitch, yaw)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def transform_vector(vector, translation, rotation):
    # Convert the vector to a quaternion to apply the rotation
    vector_quaternion = quaternion.quaternion(0, vector[0], vector[1], vector[2])
    rotation_quaternion = quaternion.quaternion(rotation[0], rotation[1], rotation[2], rotation[3])
    # rotation_quaternion = rotation_quaternion.inverse()
    
    # Apply the transformation
    rotated_vector_quaternion = rotation_quaternion * vector_quaternion * rotation_quaternion.conj()
    
    # Apply translation
    transformed_vector = rotated_vector_quaternion.vec + translation
    
    return transformed_vector

# try:
# Read data from the .dat file
data = np.genfromtxt("Output_Files/EM_Trajectory_PCNL_4.dat", delimiter=",", dtype=float)

# truth = np.genfromtxt("Output_Files/Joint_Values.dat", delimiter=",", dtype=float)

# data = np.genfromtxt("Output_Files/EM_Trajectory_Helix_good.dat", delimiter=",", dtype=float)
# truth = np.genfromtxt("Output_Files/Helix_good.csv", delimiter=",", dtype=float)


tip_ctr = data[:, 0:3] * 1.00E3
target_ctr = data[:, 3:6] * 1.00E3

# # Separate the columns into x, y, and z
hand_translation = data[:, 6:9] * 1.00E3
hand_rotation = data[:, 9:13]

# # # Compute error
error_x = abs(tip_ctr[:,0] - target_ctr[:,0])
error_y = abs(tip_ctr[:,1] - target_ctr[:,1])
error_z = abs(tip_ctr[:,2] - target_ctr[:,2])

tip_em = np.zeros(tip_ctr.shape)
target_em = np.zeros(tip_ctr.shape)
for i in range(tip_ctr.shape[0]):
    tip_em[i,:] = transform_vector(tip_ctr[i,:], hand_translation[i,:], hand_rotation[i,:])
    target_em[i,:] = transform_vector(target_ctr[i,:], hand_translation[i,:], hand_rotation[i,:])

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.scatter(
    tip_ctr[:,0], tip_ctr[:,1], tip_ctr[:,2], c="b", label="tip_in_ctr", s=20, depthshade=False
)  

ax.plot(
    target_ctr[:,0], target_ctr[:,1], target_ctr[:,2], c="r", label="target_in_ctr", linewidth=2)  # Scatter plot with red circles

# ax.plot(
#     tip_em[:,0], tip_em[:,1], tip_em[:,2], c="black", label="tip_in_em", linewidth=2)  # Scatter plot with red circles

# ax.plot(
#     target_em[:,0], target_em[:,1], target_em[:,2], c="blue", label="target_in_em", linewidth=2)  # Scatter plot with red circles

ax.set_box_aspect(
    [np.ptp(arr) for arr in [ax.get_xlim(), ax.get_ylim(), ax.get_zlim()]]
)


# Set labels for the axes using LaTeX
ax.set_xlabel(r"$\mathrm{X}$ [mm]")
ax.set_ylabel(r"$\mathrm{Y}$ [mm]")
ax.set_zlabel(r"$\mathrm{Z}$ [mm]")

# Add minor grid lines
ax.minorticks_on()
ax.grid(which="minor", color="lightgray", linestyle="--", linewidth=0.5)

# Add a title
plt.title(r"$\mathrm{EM\ Trajectory}$")

# Add a legend
ax.legend()

# Make the plot tight
plt.tight_layout()

# Show the plot
plt.show(block=False)


# Hand motion plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot(hand_translation[:,0], hand_translation[:,1], hand_translation[:,2], c="r", label="Hand", linewidth=2)  # Scatter plot with red circles
# Set labels for the axes using LaTeX
ax.set_xlabel(r"$\mathrm{X}$ [mm]")
ax.set_ylabel(r"$\mathrm{Y}$ [mm]")
ax.set_zlabel(r"$\mathrm{Z}$ [mm]")
plt.show(block=False)


# Error Distribution Plots
plt.figure(figsize=(12, 6))

# Histogram
plt.subplot(1, 3, 1)
plt.hist(error_x, bins=30, color="blue", alpha=0.7, label="Error X")
plt.hist(error_y, bins=30, color="red", alpha=0.7, label="Error Y")
plt.hist(error_z, bins=30, color="green", alpha=0.7, label="Error Z")
plt.xlabel("Error [mm]")
plt.ylabel("Frequency")
plt.legend()
plt.title("Error Histograms")

# Boxplot
plt.subplot(1, 3, 2)
sns.boxplot(data=[error_x, error_y, error_z], palette=["blue", "red", "green"])
plt.xticks([0, 1, 2], ["Error X", "Error Y", "Error Z"])
plt.title("Error Boxplot")

# Violin plot
plt.subplot(1, 3, 3)
sns.violinplot(data=[error_x, error_y, error_z], palette=["blue", "red", "green"])
plt.xticks([0, 1, 2], ["Error X", "Error Y", "Error Z"])
plt.title("Error Violin Plot")

plt.tight_layout()
plt.show()


# except FileNotFoundError:
#     print("The file 'EM_Trajectory.dat' was not found.")
# except ValueError:
#     print("Error: Unable to parse the data. Please check the format in the file.")
# except Exception as e:
#     print(f"An error occurred: {e}")
