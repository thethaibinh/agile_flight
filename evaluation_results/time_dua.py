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
            times = [item['time_to_finish'] for item in data.values() if item['Success'] == True and np.isnan(item['time_to_finish']) == False and item['policy'] == policy_name]
            if not times:
                print(f"No data found for policy {policy_name} in file {file_name}")
            return np.array(times)
        except yaml.YAMLError as exc:
            print(exc)
            return np.array([])

# Load data for each policy
dua_times = load_data("DUA.yaml", "DUA")
pyramid_normal_times = load_data("pyramid_normal_margin.yaml", "pyramid_normal")
pyramid_wide_times = load_data("pyramid_wide_margin.yaml", "pyramid_wide")

# Function to safely calculate mean and std, returns (0, 0) if array is empty
def safe_stats(data):
    if data.size > 0:
        return (data.mean(), data.std())
    else:
        return (0, 0)

# Calculate means and standard deviations safely
means = tuple(map(lambda x: safe_stats(x)[0], [dua_times, pyramid_normal_times, pyramid_wide_times]))
std_devs = tuple(map(lambda x: safe_stats(x)[1], [dua_times, pyramid_normal_times, pyramid_wide_times]))

# Plotting
ind = np.arange(len(means))  # Adjusted to dynamically set the indices based on the number of data points
width = 0.5  # the width of the bars

fig, ax = plt.subplots()
rects = ax.bar(ind, means, width, yerr=std_devs, color=['lightblue', 'lightgreen', 'lightcoral'], error_kw=dict(lw=5, capsize=5, capthick=3))

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Time to Finish [s]')
ax.set_title('Completion time by policy')
ax.set_xticks(ind)
ax.set_xticklabels(['DUA', 'RAPIDDS (normal margin)', 'RAPIDDS (wide margin)'])

# Annotate bars with time and variance values
for i, rect in enumerate(rects):
    height = rect.get_height()
    if np.isfinite(height):  # Check if the height is finite before annotating
        ax.text(rect.get_x() + rect.get_width() / 2.0, height, f'{means[i]:.2f}   ± {std_devs[i]:.2f}', ha='center', va='bottom')

plt.show()