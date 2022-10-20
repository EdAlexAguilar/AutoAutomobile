from os import listdir
import csv
import numpy as np
import itertools
import matplotlib.pyplot as plt

# find all experiment numbers available
experiments = [int(f[4:][:-4]) for f in listdir('parking')]
directory = f'parking_originaldata/'

# value options for different variables
day_options = ["day", "night"]
clarity_options = ["clear", "fog"]
ped_type_options = ["child", "adult"]
options = [day_options, clarity_options, ped_type_options]


def read_csv_as_np_dict(filename):
    data_dict = {}
    with open(filename, 'r') as file_data:
        for line in csv.reader(file_data):
            data_dict[line[0]] = line[1]
            if '[' in line[1]:
                data_list = line[1].strip('][').split(', ')
                data_dict[line[0]] = np.array([float(d) for d in data_list])
    return data_dict

crash_data = {}
for opt in itertools.product(*options):
    crash_data[str(opt).replace("'","")] = [[],[]]

for exp in experiments:
    filename = f'{directory}data{exp}.csv'
    data = read_csv_as_np_dict(filename)
    scenario_vars = f'({data["day_time"]}, {data["visibility"]}, {data["ped_type"]})'
    crash_data[scenario_vars][0].append(float(data['ego_speed']))
    crash_data[scenario_vars][1].append(bool(data['crash']=='True'))

f, axes = plt.subplots(2, 4, sharex=True, sharey=True)
for i, opt in enumerate(itertools.product(*options)):
    opt_key = str(opt).replace("'","")
    opt_data = crash_data[opt_key]

    axes[int(i>3), i%4].plot(opt_data[0], opt_data[1], '.')
    axes[int(i>3), i%4].set_title(opt_key)

plt.xticks(np.arange(20, 50,8)/3.6)
plt.show()



