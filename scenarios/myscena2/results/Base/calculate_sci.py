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

def calculate_packet_error_ratio_sci():
    pdr_vector = 'sciDecoded:vector' 
    pdr_dist_vector = 'txRxDistanceSCI:vector'
    failedHalfDuplex_vector = 'sciFailedHalfDuplex:vector'
    failedDueToProp_vector = 'sciFailedDueToProp:vector'
    failedDueToInterference_vector = 'sciFailedDueToInterference:vector'
    sciUnsensed_vector = 'sciUnsensed:vector'

    distances = df[(df["name"] == pdr_dist_vector) & (df["vectime"].notnull())]
    decoded = df[(df["name"] == pdr_vector) & (df["vectime"].notnull())]    
    halfDuplex = df[(df["name"] == failedHalfDuplex_vector) & (df["vectime"].notnull())]
    prop = df[(df["name"] == failedDueToProp_vector) & (df["vectime"].notnull())]
    interference = df[(df["name"] == failedDueToInterference_vector) & (df["vectime"].notnull())]
    sciUnsensed = df[(df["name"] == sciUnsensed_vector) & (df["vectime"].notnull())]

    distances = distances[["module", "vectime", "vecvalue"]] #各ノードのパケットの送信元との距離
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 
    distances.rename(columns={"vectime": "time"}, inplace=True)

    decoded = decoded[["module", "vecvalue"]] #各ノードのパケット受信の成功の有無(1 or 0 or -1)
    decoded.rename(columns={"vecvalue": "decode"}, inplace=True)

    halfDuplex = halfDuplex[["module", "vecvalue"]] 
    halfDuplex.rename(columns={"vecvalue": "halfDuplex"}, inplace=True)

    prop = prop[["module", "vecvalue"]] 
    prop.rename(columns={"vecvalue": "prop"}, inplace=True)

    interference = interference[["module", "vecvalue"]] 
    interference.rename(columns={"vecvalue": "interference"}, inplace=True)   

    sciUnsensed = sciUnsensed[["module", "vecvalue"]] 
    sciUnsensed.rename(columns={"vecvalue": "sciUnsensed"}, inplace=True)  

    # print(distances)
    # print(decoded)
    new_df = pd.merge(distances, decoded, on='module', how='inner')
    # print(new_df)
    new_df = pd.merge(new_df, halfDuplex, on='module', how='inner')
    # print(new_df)
    new_df = pd.merge(new_df, prop, on='module', how='inner')
    new_df = pd.merge(new_df, interference, on='module', how='inner')
    new_df = pd.merge(new_df, sciUnsensed, on='module', how='inner')
    
    # print(new_df)
    bins = []
    for i in range(100):
        bins.append({"count": 0.0, "fail": 0.0, "halfDuplex": 0.0, "prop": 0.0, "interference": 0.0, "sciUnsensed": 0.0})
    for row in new_df.itertuples():
        for i in range(len(row.distance)):
            if i == 1156:
                break
            if row.distance[i] < 1000:
                remainder = int(row.distance[i] // 10)
                if row.decode[i] >= 0:
                    bins[remainder]["count"] += 1
                    bins[remainder]["fail"] += (row.decode[i] == 0)
                    bins[remainder]['halfDuplex'] += row.halfDuplex[i]
                    bins[remainder]['prop'] += row.prop[i]
                    bins[remainder]['interference'] += row.interference[i]
                    bins[remainder]["sciUnsensed"] += row.sciUnsensed[i]

    # print(bins)
    pers = []
    distances = []
    halfDuplex_pers = []
    prop_pers = []
    interference_pers = []
    sciUnsensed_pers = []
    distance = 0
    for dictionary in bins:
        if dictionary["count"] == 0:
            pers.append(0)
            halfDuplex_pers.append(0)
            prop_pers.append(0)
            interference_pers.append(0)
            sciUnsensed_pers.append(0)
        else:
            pers.append((dictionary["fail"] / dictionary["count"]))
            halfDuplex_pers.append((dictionary["halfDuplex"] / dictionary["count"]))
            prop_pers.append((dictionary["prop"] / dictionary["count"]))
            interference_pers.append((dictionary["interference"] / dictionary["count"]))
            sciUnsensed_pers.append((dictionary["sciUnsensed"] / dictionary["count"]))
        distances.append(distance)
        distance += 10

    # print(pers)
    c1, c2, c3, c4, c5 = "black", "green", "red", "blue", "orange"
    l1, l2, l3, l4, l5 = "sci error ratio", "halfDuplex error", "propagation error", "interference error", "sciUnsensd error"
    fig, ax = plt.subplots()
    ax.plot(distances, pers, color=c1, label=l1)
    ax.plot(distances, halfDuplex_pers, color=c2, label=l2)
    ax.plot(distances, prop_pers, color=c3, label=l3)
    ax.plot(distances, interference_pers, color=c4, label=l4)
    ax.plot(distances, sciUnsensed_pers, color=c5, label=l5)
    ax.set(xlabel='Distance (m)', ylabel="SCI Error Ratio")
    ax.legend(loc="upper left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, (max(distances) + 1)])
    ax.set_ylim([0, 1.1])
    plt.xticks(np.arange(0, (max(distances))+50, step=50))
    plt.yticks(np.arange(0, 1.1, step=0.1))
    plt.savefig(args[1] + "_PER_SCI.png", dpi=300)
    # plt.show()
    plt.close(fig)

calculate_packet_error_ratio_sci()