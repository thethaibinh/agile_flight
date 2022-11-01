import matplotlib.pyplot as plt
import yaml
import numpy as np

with open("results_easy.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_easy_items_evaluated = 0
        number_easy_items_fy = 0
        number_easy_items_db = 0
        number_easy_items = 0
        for i in items:
            number_easy_items += 1
            if i[1]['Success'] == True:
                number_easy_items_evaluated += 1
                if i[1]['policy'] == 'fixed_yawing':
                    number_easy_items_fy += 1
                elif i[1]['policy'] == 'depth_based':
                    number_easy_items_db += 1
        distance_easy_fy = np.zeros(number_easy_items_fy)
        distance_easy_db = np.zeros(number_easy_items_db)
        
        number_easy_items_fy = 0
        number_easy_items_db = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'fixed_yawing':
                    distance_easy_fy[number_easy_items_fy] = i[1]['travelled_distance']
                    number_easy_items_fy += 1
                elif i[1]['policy'] == 'depth_based':
                    distance_easy_db[number_easy_items_db] = i[1]['travelled_distance']
                    number_easy_items_db += 1
    except yaml.YAMLError as exc:
        print(exc)

with open("results_medium.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_medium_items_evaluated = 0
        number_medium_items_fy = 0
        number_medium_items_db = 0
        number_medium_items = 0
        for i in items:
            number_medium_items += 1
            if i[1]['Success'] == True:
                number_medium_items_evaluated += 1
                if i[1]['policy'] == 'fixed_yawing':
                    number_medium_items_fy += 1
                elif i[1]['policy'] == 'depth_based':
                    number_medium_items_db += 1
        distance_medium_fy = np.zeros(number_medium_items_fy)
        distance_medium_db = np.zeros(number_medium_items_db)
        
        number_medium_items_fy = 0
        number_medium_items_db = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'fixed_yawing':
                    distance_medium_fy[number_medium_items_fy] = i[1]['travelled_distance']
                    number_medium_items_fy += 1
                elif i[1]['policy'] == 'depth_based':
                    distance_medium_db[number_medium_items_db] = i[1]['travelled_distance']
                    number_medium_items_db += 1
    except yaml.YAMLError as exc:
        print(exc)

with open("results_hard.yaml", "r") as stream:
    try:
        data = yaml.safe_load(stream)
        items = list(data.items())
        number_hard_items_evaluated = 0
        number_hard_items_fy = 0
        number_hard_items_db = 0
        number_hard_items = 0
        for i in items:
            number_hard_items += 1
            if i[1]['Success'] == True:
                number_hard_items_evaluated += 1
                if i[1]['policy'] == 'fixed_yawing':
                    number_hard_items_fy += 1
                elif i[1]['policy'] == 'depth_based':
                    number_hard_items_db += 1
        distance_hard_fy = np.zeros(number_hard_items_fy)
        distance_hard_db = np.zeros(number_hard_items_db)
        
        number_hard_items_fy = 0
        number_hard_items_db = 0
        for i in items:
            if i[1]['Success'] == True:
                if i[1]['policy'] == 'fixed_yawing':
                    distance_hard_fy[number_hard_items_fy] = i[1]['travelled_distance']
                    number_hard_items_fy += 1
                elif i[1]['policy'] == 'depth_based':
                    distance_hard_db[number_hard_items_db] = i[1]['travelled_distance']
                    number_hard_items_db += 1
    except yaml.YAMLError as exc:
        print(exc)

easy_distance_means = (distance_easy_fy.mean(),
                    distance_easy_db.mean())
easy_distance_std = (distance_easy_fy.std(),
                distance_easy_db.std())

medium_distance_means = (distance_medium_fy.mean(),
                    distance_medium_db.mean())
medium_distance_std = (distance_medium_fy.std(),
                distance_medium_db.std())

hard_distance_means = (distance_hard_fy.mean(),
                    distance_hard_db.mean())
hard_distance_std = (distance_hard_fy.std(),
                distance_hard_db.std())

ind = np.arange(len(easy_distance_means))  # the x locations for the groups
width = 0.24  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(ind - width, easy_distance_means, width, yerr=easy_distance_std,
                label='Easy')
rects2 = ax.bar(ind, medium_distance_means, width, yerr=medium_distance_std,
                label='Medium')
rects3 = ax.bar(ind + width, hard_distance_means, width, yerr=hard_distance_std,
                label='Hard')
# Add some text for labels, title and custom x-axis tick labels, etc.
plt.rcParams['font.sans-serif'] = "Times New Roman"
plt.rcParams['font.family'] = "sans-serif"
ax.set_ylabel('Distance [meter]')
ax.set_title('Distance travelled by policy')
ax.set_xticks(ind)
ax.set_xticklabels(('Fixed Yawing (Lee et al.)', 'Depth-based Steering (our)'))
ax.legend()

def autolabel(rects, xpos='center'):
    """
    Attach a text label above each bar in *rects*, displaying its height.

    *xpos* indicates which side to place the text w.r.t. the center of
    the bar. It can be one of the following {'center', 'right', 'left'}.
    """

    ha = {'center': 'center', 'right': 'right', 'left': 'left'}
    offset = {'center': 0, 'right': 1, 'left': -1}

    for rect in rects:
        height = rect.get_height()
        ax.annotate('{:.1f}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, 0),
                    xytext=(offset[xpos]*10, 0),  # use 3 points offset
                    textcoords="offset points",  # in both directions
                    ha=ha[xpos], va='bottom')

autolabel(rects1, "left")
autolabel(rects3, "center")

fig.tight_layout()
plt.ylim([21, 26])
plt.grid()
plt.show()