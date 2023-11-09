import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D

try:
    # Read data from the .dat file
    data = np.genfromtxt('EM_Trajectory.dat', delimiter=',', dtype=float)
    dataTraj = np.genfromtxt('../Trajectories/Square.csv', delimiter=',', dtype=float)

    # Separate the columns into x, y, and z
    x = data[:, 0] * 1.00E3
    y = data[:, 1] * 1.00E3
    z = data[:, 2] * 1.00E3

    # Data points of ground truth
    x2 = dataTraj[:, 0] * 1.00E3
    y2 = dataTraj[:, 1] * 1.00E3
    z2 = dataTraj[:, 2] * 1.00E3

    # Compute error
    error_x = x - x2
    error_y = y - y2
    error_z = z - z2

    # Create a 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(x, y, z, c='b', label='Controlled Trajectory', s=20, depthshade=False) # Scatter plot with blue bullet markers
    ax.plot(x2, y2, z2, c='r', label='Ground Truth', linewidth=2)                     # Scatter plot with red circles

    # Set labels for the axes using LaTeX
    ax.set_xlabel(r'$\mathrm{X\ Label}$ [mm]')
    ax.set_ylabel(r'$\mathrm{Y\ Label}$ [mm]')
    ax.set_zlabel(r'$\mathrm{Z\ Label}$ [mm]')

    # Add minor grid lines
    ax.minorticks_on()
    ax.grid(which='minor', color='lightgray', linestyle='--', linewidth=0.5)

    # Add a title
    plt.title(r'$\mathrm{EM\ Trajectory}$')

    # Add a legend
    ax.legend()

    # Make the plot tight
    plt.tight_layout()

    # Show the plot
    plt.show()

    # Error Distribution Plots
    plt.figure(figsize=(12, 6))

    # Histogram
    plt.subplot(1, 3, 1)
    plt.hist(error_x, bins=30, color='blue', alpha=0.7, label='Error X')
    plt.hist(error_y, bins=30, color='red', alpha=0.7, label='Error Y')
    plt.hist(error_z, bins=30, color='green', alpha=0.7, label='Error Z')
    plt.xlabel('Error [mm]')
    plt.ylabel('Frequency')
    plt.legend()
    plt.title('Error Histograms')

    # Boxplot
    plt.subplot(1, 3, 2)
    sns.boxplot(data=[error_x, error_y, error_z], palette=['blue', 'red', 'green'])
    plt.xticks([0, 1, 2], ['Error X', 'Error Y', 'Error Z'])
    plt.title('Error Boxplot')

    # Violin plot
    plt.subplot(1, 3, 3)
    sns.violinplot(data=[error_x, error_y, error_z], palette=['blue', 'red', 'green'])
    plt.xticks([0, 1, 2], ['Error X', 'Error Y', 'Error Z'])
    plt.title('Error Violin Plot')

    plt.tight_layout()
    plt.show()

except FileNotFoundError:
    print("The file 'EM_Trajectory.dat' was not found.")
except ValueError:
    print("Error: Unable to parse the data. Please check the format in the file.")
except Exception as e:
    print(f"An error occurred: {e}")
