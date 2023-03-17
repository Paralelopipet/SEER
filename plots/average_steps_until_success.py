from seaborn import heatmap
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

filepath = str(Path(__file__).parent / "Wandb data.xlsx")
sheetname = "tip over count"
# ds = pd.read_csv(filepath)
ds = pd.read_excel(filepath,sheetname)

ds[['environment', 'controller']] = ds['Name'].str.split('-', 1, expand=True)

envs = ds['environment']
conts = ds['controller']
avg = ds['avg']
envs_name = []
conts_name = []
tip_over_table = np.zeros([8,9])
for i in range(len(ds['avg'])):
    # get index of name
    if(envs[i][4:] in envs_name):
        env_ind = envs_name.index(envs[i][4:])
    else:
        env_ind = len(envs_name)
        envs_name = envs_name + [envs[i][4:]]
    # get index of conts
    if(conts[i] in conts_name):
        cont_ind = conts_name.index(conts[i])
    else:
        cont_ind = len(conts_name)
        conts_name = conts_name + [conts[i]]
    tip_over_table[env_ind, cont_ind] = float(round(avg[i], 2))

heatmap(tip_over_table,annot=[[f'{float(f"{tip_over_table[i2,i1]:.2g}"):g}' for i1 in range(len(tip_over_table[i2]))] for i2 in range(len(tip_over_table))],fmt="s", xticklabels=conts_name,yticklabels= envs_name, cmap="rocket_r", cbar_kws = dict(location="top"))
plt.show()
