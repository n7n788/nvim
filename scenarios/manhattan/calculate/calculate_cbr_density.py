# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import re

#文字列を数字、True, False, Noneのいずれかに変換する関数
def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

# 数値に変換する関数
def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

args = sys.argv
df1 = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

df2 = pd.read_csv(args[2], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

df3 = pd.read_csv(args[3], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

# データフレームを入力として、時間ごとのCBRの平均を返す
def getCbr(df):
    cbr_vector = "cbr:vector"
    cbr_df = df[(df["name"] == cbr_vector) & (df["vectime"].notnull())]
    cbr_df = cbr_df[["module", "vecvalue", "vectime"]]

    bins = {"cbr": [], "time": []}
    counts = []
    simulation_time = 50
    for i in range(simulation_time * 10):
        bins["cbr"].append(0)
        counts.append(0)

    for row in cbr_df.itertuples():
        for i in range(len(row.vectime)):
            if row.vectime[i] >= simulation_time:
                continue
            # 0.1秒ごとにcbrの平均を取る
            remainder = int(row.vectime[i] * 10)
            bins["cbr"][remainder] += row.vecvalue[i]
            counts[remainder] += 1
    
    for i in range(len(bins["cbr"])):
        if counts[i] == 0:
            bins["cbr"][i] == 0
        else:
            bins["cbr"][i] /= counts[i]
    
    time = 0
    for i in range(simulation_time * 10):
        bins["time"].append(time)
        time += 0.1

    return bins

cbr1 = getCbr(df1)
cbr2 = getCbr(df2)
cbr3 = getCbr(df3)

l1, l2, l3 = "100 veh/km^2 ", "200 veh/km^2", "300 veh/km^2"
w1, w2, w3 = 1, 1, 1

fig, ax = plt.subplots()
ax.plot(cbr1["time"], cbr1["cbr"], label=l1, linewidth=w1)
ax.plot(cbr2["time"], cbr2["cbr"], label=l2, linewidth=w2)
ax.plot(cbr3["time"], cbr3["cbr"], label=l3, linewidth=w3)

ax.set(xlabel='Simulation Time (s)', ylabel="Channel Busy Ratio")
ax.legend(loc="lower left")
ax.tick_params(direction='in')
ax.set_xlim([0, (max(cbr1["time"]) + 1)])
ax.set_ylim([0, 1.1])
plt.xticks(np.arange(0, (max(cbr1["time"]))+5, step=5))
plt.yticks(np.arange(0, 1.1, step=0.1))
plt.savefig("CBR_density.png", dpi=300)
# plt.show()
plt.close(fig)