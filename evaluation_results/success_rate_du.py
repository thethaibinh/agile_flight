import yaml
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

# Set the font globally to Times New Roman, size 12
matplotlib.rcParams['font.family'] = 'Times New Roman'
matplotlib.rcParams['font.size'] = 20

def load_data(file_name):
    with open(file_name, 'r') as file:
        data = yaml.safe_load(file)

    success_count = 0
    collision_count = 0
    incomplete_count = 0

    for rollout in data.values():
        if rollout['number_crashes'] > 0:
            collision_count += 1
        elif rollout['Success'] == False and rollout['number_crashes'] == 0:
            incomplete_count += 1
        elif rollout['Success'] == True:
            success_count += 1

    total_trials = success_count + collision_count + incomplete_count
    return (success_count / total_trials * 100, collision_count / total_trials * 100, incomplete_count / total_trials * 100)

# Load data from files
du_results = load_data('DU.yaml')
pyramid_narrow_results = load_data('pyramid_narrow_margin.yaml')
pyramid_wide_results = load_data('pyramid_wide_margin.yaml')

# Data to plot
bar_width = 0.5  # the width of the bars
group_width = 1  # space between groups
index = np.array([0, 1, 2, 3 + group_width, 4 + group_width, 5 + group_width])


success_forest = (du_results[0], pyramid_narrow_results[0], pyramid_wide_results[0])
collisions_forest = (du_results[1], pyramid_narrow_results[1], pyramid_wide_results[1])
incomplete_forest = (du_results[2], pyramid_narrow_results[2], pyramid_wide_results[2])

# Narrow gap environment
success_narrow = (99.4, 0.0, 0.0)
collisions_narrow = (0.3, 0.0, 0.0)
incomplete_narrow = (0.4, 100.0, 100.0)

# Create plot
fig, ax = plt.subplots()

# Adding text labels inside the bars
def add_labels(bars):
    for bar in bars:
        height = bar.get_height()
        if height > 0:  # Only add labels if the height is greater than 0
            # Adjust text position for visibility within the bar
            text_position = bar.get_y() + height / 2.5
            if height > 90:  # Adjust for bars that are too high
                text_position = 91  # Lower the text position
            ax.annotate(f'{height:.2f}%',
                        xy=(bar.get_x() + bar.get_width() / 2, text_position),
                        xytext=(0, 0),  # No offset
                        textcoords="offset points",
                        ha='center', va='center',  # Center vertically and horizontally
                        color='black')  # Change text color for visibility

# Function to label groups
def label_groups(position, text):
    ax.text(position, ax.get_ylim()[1] - 4.5, text, ha='center', va='top')

# Plotting bars and adding labels inside them
# Forest environment
rects1 = ax.bar(index[:3], success_forest, bar_width, label='Success', color='lightgreen')
rects2 = ax.bar(index[:3], collisions_forest, bar_width, bottom=success_forest, label='Collision', color='salmon')
rects3 = ax.bar(index[:3], incomplete_forest, bar_width, bottom=np.array(success_forest) + np.array(collisions_forest), label='Incomplete', color='lightblue')

add_labels(rects1)
add_labels(rects2)
add_labels(rects3)

# Narrow gap environment
rects4 = ax.bar(index[3:], success_narrow, bar_width, color='lightgreen')
rects5 = ax.bar(index[3:], collisions_narrow, bar_width, bottom=success_narrow, color='salmon')
rects6 = ax.bar(index[3:], incomplete_narrow, bar_width, bottom=np.array(success_narrow) + np.array(collisions_narrow), color='lightblue')

add_labels(rects4)
add_labels(rects5)
add_labels(rects6)

# Labeling groups
label_groups(np.mean(index[:3]), 'Forest scenario')
label_groups(np.mean(index[3:]), 'Narrow gap scenario')

ax.set_xlabel('Policy')
ax.set_ylabel('Percentage (%)')
# ax.set_title('Results by policy and environment')
ax.set_xticks(index)  # Center the labels between the groups
ax.set_xticklabels(['DUA', 'RAPPIDS \n normal margin', 'RAPPIDS \n wide margin', 'DUA', 'RAPPIDS \n narrow margin', 'RAPPIDS \n wide margin'])
ax.set_ylim([90, 100])  # Set y-axis limits from 0% to 110% to accommodate group labels
ax.legend()

plt.tight_layout()  # Adjust layout to not overlap
plt.show()