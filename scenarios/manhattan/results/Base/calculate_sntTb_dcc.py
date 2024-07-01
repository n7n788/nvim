# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import re
import statistics

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

simulation_time = 50
max_tb_cnt = 0
def getSentCnt(df):
    global max_tb_cnt
    tb_sent_vector = "tbSent:vector"
    
    tb_sent = df[(df["name"] == tb_sent_vector) & (df["vectime"].notnull())]
    
    tb_sent = tb_sent[["module", "vectime"]]
    
    bins = {"tb": [], "time": []}
    time = 0
    for i in range(simulation_time):
        bins["tb"].append(0)
        bins["time"].append(time)
        time += 1 
    for row in tb_sent.itertuples():
        for i in range(len(row.vectime)):
            if row.vectime[i] < 50:
                remainer = int(row.vectime[i])
                bins["tb"][remainer] += 1
                max_tb_cnt = max(max_tb_cnt, bins["tb"][remainer])
    return bins

sent_cnt1 = getSentCnt(df1)
sent_cnt2 = getSentCnt(df2)
# print(sent_cnt1)
sent_cnts = [sent_cnt1, sent_cnt2]
labels = ["No Dcc", "Dcc Reactive"]

# シミュレーション時間×1秒間に受信したtbの個数
fig, ax = plt.subplots()
for i in range(len(sent_cnts)):
    ax.plot(sent_cnts[i]["time"], sent_cnts[i]["tb"], label=labels[i])
ax.set(xlabel="Simulation Time [s]", ylabel="Sent tb Count")
ax.legend(loc="upper right")
ax.tick_params(direction='in')
ax.set_xlim([0, simulation_time + 1])
ax.set_ylim([0, max_tb_cnt + 1])
plt.xticks(np.arange(0, simulation_time + 5, step=5))
# plt.yticks(np.arange(0, max_cam_cnt)
plt.savefig(args[1] + "_Tb_sent_count_dcc.png", dpi=300)
plt.close(fig)
