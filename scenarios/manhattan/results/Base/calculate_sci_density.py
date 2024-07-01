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

# シミュレーション結果全体のデータフレームを受信し、SCIに関して抽出したデータフレームを返す
def getSci(df):
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

    new_df = pd.merge(distances, decoded, on='module', how='inner')
    new_df = pd.merge(new_df, halfDuplex, on='module', how='inner')
    new_df = pd.merge(new_df, prop, on='module', how='inner')
    new_df = pd.merge(new_df, interference, on='module', how='inner')
    new_df = pd.merge(new_df, sciUnsensed, on='module', how='inner')

    return new_df

def getPers(new_df):
    bins = []
    for i in range(100):
        bins.append({"count": 0.0, "fail": 0.0, "halfDuplex": 0.0, "prop": 0.0, "interference": 0.0, "sciUnsensed": 0.0})
    for row in new_df.itertuples():
        for i in range(len(row.distance)):
            if row.distance[i] < 1000:
                remainder = int(row.distance[i] // 10)
                if row.decode[i] >= 0:
                    bins[remainder]["count"] += 1
                    bins[remainder]["fail"] += (row.decode[i] == 0)
                    bins[remainder]['halfDuplex'] += row.halfDuplex[i]
                    bins[remainder]['prop'] += row.prop[i]
                    bins[remainder]['interference'] += row.interference[i]
                    bins[remainder]["sciUnsensed"] += row.sciUnsensed[i]

    
    pers = {"distance": [], "all": [], "halfDuplex": [], "prop": [], "interference": [], "sciUnsensed": []}
    distance = 0
    for dictionary in bins:
        if dictionary["count"] == 0:
            pers["distance"].append(0)
            pers["all"].append(0)
            pers["prop"].append(0)
            pers["interference"].append(0)
            pers["sciUnsensed"].append(0)
        else:
            pers["all"].append((dictionary["fail"] / dictionary["count"]))
            pers["halfDuplex"].append((dictionary["halfDuplex"] / dictionary["count"]))
            pers["prop"].append((dictionary["prop"] / dictionary["count"]))
            pers["interference"].append((dictionary["interference"] / dictionary["count"]))
            pers["sciUnsensed"].append((dictionary["sciUnsensed"] / dictionary["count"]))
        pers["distance"].append(distance)
        distance += 10

    return pers

max_distance = 1000
new_df1 = getSci(df1)
new_df2 = getSci(df2)
new_df3 = getSci(df3)
pers1 = getPers(new_df1)
pers2 = getPers(new_df2)
pers3 = getPers(new_df3)

l1, l2, l3 = "100 veh/km^2 ", "200 veh/km^2", "300 veh/km^2"
w1, w2, w3 = 1, 1, 1

fig, ax = plt.subplots()
ax.plot(pers1["distance"], pers1["all"], label=l1, linewidth=w1)
ax.plot(pers2["distance"], pers2["all"], label=l2, linewidth=w2)
ax.plot(pers3["distance"], pers3["all"], label=l3, linewidth=w3)
ax.set(xlabel='Distance (m)', ylabel="SCI Error Ratio")
ax.legend(loc="upper left")
ax.tick_params(direction='in')
ax.set_xlim([0,  max_distance + 1])
ax.set_ylim([0, 1.1])
plt.xticks(np.arange(0, max_distance + 50, step=50))
plt.yticks(np.arange(0, 1.1, step=0.1))
plt.savefig("PER_SCI_density.png", dpi=300)
# plt.show()
plt.close(fig)

fig, ax = plt.subplots()
ax.plot(pers1["distance"], pers1["halfDuplex"], label=l1, linewidth=w1)
ax.plot(pers2["distance"], pers2["halfDuplex"], label=l2, linewidth=w2)
ax.plot(pers3["distance"], pers3["halfDuplex"], label=l3, linewidth=w3)
ax.set(xlabel='Distance (m)', ylabel="SCI HalfDuplex Error Ratio")
ax.legend(loc="upper left")
ax.tick_params(direction='in')
ax.set_xlim([0,  max_distance + 1])
ax.set_ylim([0, 1.1])
plt.xticks(np.arange(0, max_distance + 50, step=50))
plt.yticks(np.arange(0, 1.1, step=0.1))
plt.savefig("PER_SCI_halfDuplex_density.png", dpi=300)
# plt.show()
plt.close(fig)

fig, ax = plt.subplots()
ax.plot(pers1["distance"], pers1["prop"], label=l1, linewidth=w1)
ax.plot(pers2["distance"], pers2["prop"], label=l2, linewidth=w2)
ax.plot(pers3["distance"], pers3["prop"], label=l3, linewidth=w3)
ax.set(xlabel='Distance (m)', ylabel="SCI Propagation Error Ratio")
ax.legend(loc="upper left")
ax.tick_params(direction='in')
ax.set_xlim([0,  max_distance + 1])
ax.set_ylim([0, 1.1])
plt.xticks(np.arange(0, max_distance + 50, step=50))
plt.yticks(np.arange(0, 1.1, step=0.1))
plt.savefig("PER_SCI_propagation_density.png", dpi=300)
# plt.show()
plt.close(fig)

fig, ax = plt.subplots()
ax.plot(pers1["distance"], pers1["interference"], label=l1, linewidth=w1)
ax.plot(pers2["distance"], pers2["interference"], label=l2, linewidth=w2)
ax.plot(pers3["distance"], pers3["interference"], label=l3, linewidth=w3)
ax.set(xlabel='Distance (m)', ylabel="SCI Interference Error Ratio")
ax.legend(loc="upper left")
ax.tick_params(direction='in')
ax.set_xlim([0,  max_distance + 1])
ax.set_ylim([0, 1.1])
plt.xticks(np.arange(0, max_distance + 50, step=50))
plt.yticks(np.arange(0, 1.1, step=0.1))
plt.savefig("PER_SCI_interference_density.png", dpi=300)
# plt.show()
plt.close(fig)

fig, ax = plt.subplots()
ax.plot(pers1["distance"], pers1["sciUnsensed"], label=l1, linewidth=w1)
ax.plot(pers2["distance"], pers2["sciUnsensed"], label=l2, linewidth=w2)
ax.plot(pers3["distance"], pers3["sciUnsensed"], label=l3, linewidth=w3)
ax.set(xlabel='Distance (m)', ylabel="SCI unsensed Error Ratio")
ax.legend(loc="upper left")
ax.tick_params(direction='in')
ax.set_xlim([0,  max_distance + 1])
ax.set_ylim([0, 1.1])
plt.xticks(np.arange(0, max_distance + 50, step=50))
plt.yticks(np.arange(0, 1.1, step=0.1))
plt.savefig("PER_SCI_unsensed_density.png", dpi=300)
# plt.show()
plt.close(fig)