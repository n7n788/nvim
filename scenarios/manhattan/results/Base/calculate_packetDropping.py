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

simulation_time = 50
max_drop_cnt = 0

def getPacketDropping(df):
    global max_drop_cnt
    
    packet_drop_vector = "packetDropDcc:vector"   
    packet_drop = df[(df["name"] == packet_drop_vector) & (df["vectime"].notnull())]
    packet_drop = packet_drop[["module", "vectime"]]
    
    bins = {"drop": [], "time": []}
    time = 0
    for i in range(simulation_time):
        bins["drop"].append(0)
        bins["time"].append(time)
        time += 1 
    for row in packet_drop.itertuples():
        for i in range(len(row.vectime)):
            if row.vectime[i] < 50:
                remainer = int(row.vectime[i])
                bins["drop"][remainer] += 1
                max_drop_cnt = max(max_drop_cnt, bins["drop"][remainder])

packet_drop1 = getPacketDropping(df1)

fig, ax = plt.subplots()
ax.plot(packet_drop1["time"], packet_drop1["drop"], label="pacekt Dropping CRLimit")
ax.set(xlabel="Simulation Time [s]", ylabel="Packet Dropping Count by CRLimit")
ax.legend(loc="upper right")
ax.tick_params(direction="in")
ax.set_xlim([0, simulation_time + 1])
ax.set_ylim([0, max_drop_cnt + 1])
plt.savefig(args[1] + "_packet_drop_byCRLimit.png", dpi=300)
plt.close(fig)