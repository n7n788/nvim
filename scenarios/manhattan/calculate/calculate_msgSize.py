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
df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

def calculate_message_size():
    packetSize_vector = 'sentPacketToLowerLayer:vector(packetBytes)'
    
    packetSize_df = df[(df["name"] == packetSize_vector) & (df["vectime"].notnull())]
    packetSize_df = packetSize_df[["module", "vecvalue", "vectime"]]
    
    bins = []
    times = []
    
    for row in packetSize_df.itertuples():
        for i in range(len(row.vectime)):
            bins.append(row.vecvalue[i])
            times.append(row.vectime[i])
        break
    # print(bins)
    # print(times)
    
    fig, ax = plt.subplots()
    ax.plot(times, bins, label="Packet Size")
    ax.set(xlabel='Simulation Time (s)', ylabel="Packet Size")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, (max(times) + 1)])
    ax.set_ylim([0, 1.1])
    plt.xticks(np.arange(0, (max(times))+5, step=5))
    plt.yticks(np.arange(0, 1.1, step=0.1))
    plt.savefig(args[1] + "_packetSize.png", dpi=300)
    # plt.show()
    plt.close(fig)

calculate_message_size()