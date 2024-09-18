import matplotlib.pyplot as plt
import yaml
import numpy as np
import matplotlib

# Set the font globally to Times New Roman, size 20
matplotlib.rcParams['font.family'] = 'Times New Roman'
matplotlib.rcParams['font.size'] = 24

def load_data(file_name, policy_name):
    with open(file_name, "r") as stream:
        try:
            data = yaml.safe_load(stream)
            distances = [item['travelled_distance'] for item in data.values() if item['Success'] == True and item['policy'] == policy_name]
            if not distances:
                print(f"No data found for policy {policy_name} in file {file_name}")
            return np.array(distances)
        except yaml.YAMLError as exc:
            print(exc)
            return np.array([])

# Load data for each policy
dua_distances = load_data("DUA.yaml", "DUA")
pyramid_normal_distances = load_data("pyramid_normal_margin.yaml", "pyramid_normal")
pyramid_wide_distances = load_data("pyramid_wide_margin.yaml", "pyramid_wide")

# Function to safely calculate mean and std, returns (0, 0) if array is empty
def safe_stats(data):
    if data.size > 0:
        return (data.mean(), data.std())
    else:
        return (0, 0)

# Calculate means and standard deviations safely
means = tuple(map(lambda x: safe_stats(x)[0], [dua_distances, pyramid_normal_distances, pyramid_wide_distances]))
std_devs = tuple(map(lambda x: safe_stats(x)[1], [dua_distances, pyramid_normal_distances, pyramid_wide_distances]))

# Plotting
ind = np.arange(3)  # the x locations for the groups
width = 0.5  # the width of the bars

fig, ax = plt.subplots()
rects = ax.bar(ind, means, width, yerr=std_devs, color=['lightblue', 'lightgreen', 'lightcoral'], error_kw=dict(lw=5, capsize=5, capthick=3))

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Distance [m]')
ax.set_title('Travelled distance by policy')
ax.set_xticks(ind)
ax.set_xticklabels(['DUA', 'RAPIDDS (normal margin)', 'RAPIDDS (wide margin)'])

# Adding legends similar to success_rate_du.py
# ax.legend(['DUA', 'RAPIDDS Normal', 'RAPIDDS Wide'], title="Policies")

# Annotate bars with distance and variance values
for i, rect in enumerate(rects):
    height = rect.get_height()
    ax.text( rect.get_x() + rect.get_width() / 2.0, height, f'{means[i]:.2f}   ± {std_devs[i]:.2f}', ha='center', va='bottom')

plt.show()