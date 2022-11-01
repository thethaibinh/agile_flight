from matplotlib import pyplot as plt
import textwrap as twp
import yaml
import numpy as np

with open("results_easy.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_easy_items_fy_failed = 0
        number_easy_items_db_failed = 0
        number_easy_items_fy_success = 0
        number_easy_items_db_success = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'fixed_yawing':
                    number_easy_items_fy_success += 1
                elif i[1]['policy'] == 'depth_based':
                    number_easy_items_db_success += 1
            else:
                if i[1]['policy'] == 'fixed_yawing':
                    number_easy_items_fy_failed += 1
                elif i[1]['policy'] == 'depth_based':
                    number_easy_items_db_failed += 1
        easy_fy_success_rate = number_easy_items_fy_success / (number_easy_items_fy_success + number_easy_items_fy_failed)
        easy_db_success_rate = number_easy_items_db_success / (number_easy_items_db_success + number_easy_items_db_failed)
    except yaml.YAMLError as exc:
        print(exc)

with open("results_medium.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_medium_items_fy_failed = 0
        number_medium_items_db_failed = 0
        number_medium_items_fy_success = 0
        number_medium_items_db_success = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'fixed_yawing':
                    number_medium_items_fy_success += 1
                elif i[1]['policy'] == 'depth_based':
                    number_medium_items_db_success += 1
            else:
                if i[1]['policy'] == 'fixed_yawing':
                    number_medium_items_fy_failed += 1
                elif i[1]['policy'] == 'depth_based':
                    number_medium_items_db_failed += 1
        medium_fy_success_rate = number_medium_items_fy_success / (number_medium_items_fy_success + number_medium_items_fy_failed)
        medium_db_success_rate = number_medium_items_db_success / (number_medium_items_db_success + number_medium_items_db_failed)
    except yaml.YAMLError as exc:
        print(exc)

with open("results_hard.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_hard_items_fy_failed = 0
        number_hard_items_db_failed = 0
        number_hard_items_fy_success = 0
        number_hard_items_db_success = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'fixed_yawing':
                    number_hard_items_fy_success += 1
                elif i[1]['policy'] == 'depth_based':
                    number_hard_items_db_success += 1
            else:
                if i[1]['policy'] == 'fixed_yawing':
                    number_hard_items_fy_failed += 1
                elif i[1]['policy'] == 'depth_based':
                    number_hard_items_db_failed += 1
        hard_fy_success_rate = number_hard_items_fy_success / (number_hard_items_fy_success + number_hard_items_fy_failed)
        hard_db_success_rate = number_hard_items_db_success / (number_hard_items_db_success + number_hard_items_db_failed)
    except yaml.YAMLError as exc:
        print(exc)

# float_formatter = "{:.4f}".format
fig = plt.figure()
cells = np.round([[easy_fy_success_rate, easy_db_success_rate], 
        [medium_fy_success_rate, medium_db_success_rate], 
        [hard_fy_success_rate, hard_db_success_rate]],6)
img = plt.imshow(cells, cmap="RdYlGn")
img.set_visible(False)
plt.axis('off')
ax = fig.add_subplot(111, frameon=False, xticks = [], yticks = [])

tb = plt.table(cellText = cells, 
    cellLoc='center',
    rowLabels = ['Easy', 'Medium', 'Hard'], 
    colLabels = [twp.fill('Fixed yawing (Lee et al.)', 16),
                twp.fill('DESS (our)', 16)],
    loc = 'center',
    cellColours = img.to_rgba(cells))
for key, cell in tb.get_celld().items():
    cell.set_edgecolor('w')

tb.scale(1, 6)
tb.auto_set_font_size(False)
tb.set_fontsize(12)
ax.figure

cbar = plt.colorbar(location='top')
cbar.ax.set_xlabel('Success rate', fontsize=12)
cbar.ax.tick_params(labelsize=12)
cbar.outline.set_linewidth(0)
plt.axis('off')
plt.show()