import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

folderName = "2024-08-27_16-52-21";
# robot_data = np.genfromtxt("Output_Files/" + folderName + "/errors.csv", delimiter=",", dtype=float)

# Load the data from a CSV file
df = pd.read_csv("Output_Files/" + folderName + "/errors.csv")  # Assuming the file is named 'errors.csv'

# Calculate the Euclidean norm for each row (error vector)
df['norm'] = np.sqrt(df['x']**2 + df['y']**2 + df['z']**2) * 1e3

average_norm = df['norm'].median()

print("Average of the norms:", average_norm)

# Plotting the histogram of the norms
plt.figure(figsize=(10, 6))
plt.hist(df['norm'], bins=30, color='blue', alpha=0.7)
plt.title('Histogram of Error Norms')
plt.xlabel('Error Norm [mm]')
plt.ylabel('Frequency')
plt.grid(True)
plt.show()