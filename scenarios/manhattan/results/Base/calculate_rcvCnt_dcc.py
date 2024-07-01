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
max_cam_cnt = 0
max_cpm_cnt = 0
def getRcvCnt(df):
    global max_cam_cnt, max_cpm_cnt
    cpm_received_vector = "camReceivedGenerationTime:vector"
    cam_received_vector = "cpmReceivedGenerationTime:vector"
    
    cam_received = df[(df["name"] == cam_received_vector) & (df["vectime"].notnull())]
    cpm_received = df[(df["name"] == cpm_received_vector) & (df["vectime"].notnull())]
    
    cam_received = cam_received[["module", "vectime"]]
    cpm_received = cpm_received[["module", "vectime"]]
    
    bins = {"cam": [], "cpm": [], "time": []}
    time = 0
    for i in range(simulation_time):
        bins["cam"].append(0)
        bins["cpm"].append(0)
        bins["time"].append(time)
        time += 1 
    for row in cam_received.itertuples():
        for i in range(len(row.vectime)):
            if row.vectime[i] < 50:
                remainer = int(row.vectime[i])
                bins["cam"][remainer] += 1
                max_cam_cnt = max(max_cam_cnt, bins["cam"][remainer])
    for row in cpm_received.itertuples():
        for i in range(len(row.vectime)):
            if row.vectime[i] < 50:
                remainer = int(row.vectime[i])
                bins["cpm"][remainer] += 1
                max_cpm_cnt = max(max_cpm_cnt, bins["cpm"][remainer])
    return bins

rcv_cnt1 = getRcvCnt(df1)
rcv_cnt2 = getRcvCnt(df2)
# print(rcv_cnt1)
rcv_cnts = [rcv_cnt1, rcv_cnt2]
labels = ["No Dcc", "Dcc Reactive"]

# シミュレーション時間×1秒間に受信したCAMの個数
fig, ax = plt.subplots()
for i in range(len(rcv_cnts)):
    ax.plot(rcv_cnts[i]["time"], rcv_cnts[i]["cam"], label=labels[i])
ax.set(xlabel="Simulation Time [s]", ylabel="Received Cam Count")
ax.legend(loc="upper right")
ax.tick_params(direction='in')
ax.set_xlim([0, simulation_time + 1])
ax.set_ylim([0, max_cam_cnt + 1])
plt.xticks(np.arange(0, simulation_time + 5, step=5))
# plt.yticks(np.arange(0, max_cam_cnt)
plt.savefig(args[1] + "_Cam_received_count_dcc.png", dpi=300)
plt.close(fig)

# シミュレーション時間×1秒間に受信したCPMの個数
fig, ax = plt.subplots()
for i in range(len(rcv_cnts)):
    ax.plot(rcv_cnts[i]["time"], rcv_cnts[i]["cpm"], label=labels[i])
ax.set(xlabel="Simulation Time [s]", ylabel="Received Cpm Count")
ax.legend(loc="upper right")
ax.tick_params(direction='in')
ax.set_xlim([0, simulation_time + 1])
ax.set_ylim([0, max_cpm_cnt + 1])
plt.xticks(np.arange(0, simulation_time + 5, step=5))
# plt.yticks(np.arange(0, max_cam_cnt)
plt.savefig(args[1] + "_Cpm_received_count_dcc.png", dpi=300)
plt.close(fig)
