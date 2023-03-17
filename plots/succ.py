from seaborn import heatmap
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

filepath = str(Path(__file__).parent / "Wandb data.xlsx")
sheetname = "successful episodes rate"
# ds = pd.read_csv(filepath)
ds = pd.read_excel(filepath,sheetname)

ds[['environment', 'controller']] = ds['Name'].str.split('-', 1, expand=True)

envs = ds['environment']
conts = ds['controller']
avg = ds['avg']
envs_name = []
conts_name = []
succ_rate_table = np.zeros([8,9])
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
    succ_rate_table[env_ind, cont_ind] = round(avg[i], 2)

heatmap(succ_rate_table,annot=True,xticklabels=conts_name,yticklabels= envs_name, cbar_kws = dict(location="top"))
plt.show()