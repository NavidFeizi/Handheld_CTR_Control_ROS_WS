import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import LinearSegmentedColormap

plt.rcParams["grid.color"] = "lightgray"
plt.rcParams["grid.linewidth"] = 0.5
matplotlib.rc("font", family="serif", size=7)
matplotlib.rcParams["text.usetex"] = True


folderName = "09-14_phantom_fl_upper_1"

# Load the data from a CSV file
df_error = pd.read_csv("Output_Files/" + folderName + "/Results.csv")
df_target = pd.read_csv("Output_Files/" + folderName + "/phantomTargets.csv")
df_stone = pd.read_csv("Output_Files/" + "/Stone_vertices.csv")

# Calculate the Euclidean norm for each row (error vector)
df_error["norm"] = (
    np.sqrt(df_error["e_x"] ** 2 + df_error["e_y"] ** 2 + df_error["e_z"] ** 2) * 1e3
)

median_norm = df_error["norm"].median()
mean_norm = df_error["norm"].mean()

print("Median of the norms: {:.4f}".format(median_norm))
print("Mean of the norms: {:.4f}".format(mean_norm))

# Plotting the histogram of the norms
plt.figure(figsize=(10, 6))
plt.hist(df_error['norm'], bins=30, color='blue', alpha=0.7)
plt.title('Histogram of Error Norms')
plt.xlabel('Error Norm [mm]')
plt.ylabel('Frequency')
plt.grid(True)
plt.show()

colors = ["blue", "red"]  # Use any CSS color or hex codes
n_bins = [0, 1]  # Ranges for each color
cmap_name = "my_cmap"
custom_cmap = LinearSegmentedColormap.from_list(cmap_name, list(zip(n_bins, colors)))

# Plot
fig = plt.figure(figsize=(8, 5))
ax = fig.add_subplot(111, projection="3d")

sc = ax.scatter(
    df_stone['x'].to_numpy(),
    df_stone['y'].to_numpy(),
    df_stone['z'].to_numpy(),
    c='gray',
    s=0.4,
)  # 'viridis' is a color map you can change

sc = ax.scatter(
    df_target['tgt_x'].to_numpy()*1e3,
    df_target['tgt_y'].to_numpy()*1e3,
    df_target['tgt_z'].to_numpy()*1e3,
    c=df_error["norm"],
    cmap=custom_cmap,
    s=30,
)  # 'viridis' is a color map you can change

# Adding a color bar to show the error values
cbar = plt.colorbar(sc, pad=0.1)
cbar.set_label("Error")

# Adding labels and title
ax.set_xlabel("X Coordinate")
ax.set_ylabel("Y Coordinate")
ax.set_zlabel("Z Coordinate")
ax.set_title("3D Point Cloud Error Visualization")
# ax.set_box_aspect([1,1,1]) 
ax.set_aspect('equal')

ax.set_xlim(-10, 5)
ax.set_ylim(45, 60)
ax.set_zlim(180, 205)

plt.grid(True)
plt.show()