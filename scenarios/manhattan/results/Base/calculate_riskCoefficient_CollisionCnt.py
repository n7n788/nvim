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

min_risk = -5.0
max_risk = 5.1
risk_num = int((max_risk - min_risk) * 10) + 1
max_aoi = 0
# trueFlag = Trueの場合、真の危険係数を取得
def calculate_riskRow(df, trueFlag):
    global min_risk, max_risk, risk_num, max_aoi
    risk_vector = 'riskRow:vector'
    if trueFlag:
        risk_vector = 'trueRiskRow:vector'
    aoi_vector = 'aoi:vector'
    dist_vector = 'perceptedObjectDistance:vector'
    hash_vector = 'hashId:vector'
    sensing_vector = 'sensing:vector'
    connected_vector = 'connected:vector'
    
    risks = df[(df["name"] == risk_vector) & (df["vectime"].notnull())]
    aois = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    distances = df[(df["name"] == dist_vector) & (df["vectime"].notnull())]
    hashs = df[(df["name"] == hash_vector) & (df["vectime"].notnull())]
    sensings = df[(df["name"] == sensing_vector) & (df["vectime"].notnull())]
    connecteds = df[(df["name"] == connected_vector) & (df["vectime"].notnull())]
    
    risks = risks[["module", "vecvalue", "vectime"]]
    risks.rename(columns={"vecvalue": "risk"}, inplace=True)
    risks.rename(columns={"vectime": "time"}, inplace=True)
    
    aois = aois[["module", "vecvalue"]]
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    
    distances = distances[["module", "vecvalue"]]
    distances.rename(columns={"vecvalue": "distance"}, inplace=True)
    
    hashs = hashs[["module", "vecvalue"]]
    hashs.rename(columns={"vecvalue": "hash"}, inplace=True)
    
    sensings = sensings[["module", "vecvalue"]]
    sensings.rename(columns={"vecvalue": "sensing"}, inplace=True)
    
    connecteds = connecteds[["module", "vecvalue"]]
    connecteds.rename(columns={"vecvalue": "connected"}, inplace=True)
    
    new_df = pd.merge(risks, aois, on="module", how="inner")
    new_df = pd.merge(new_df, distances, on="module", how="inner")
    new_df = pd.merge(new_df, hashs, on="module", how="inner")
    new_df = pd.merge(new_df, sensings, on="module", how="inner")
    new_df = pd.merge(new_df, connecteds, on="module", how="inner")
    
    sum = 0
    for row in new_df.itertuples():
        over_risk_hashs =  set() # 危険係数がオーバーした車両のid
        for i in range(len(row.risk)):
            if (row.distance[i] < 300) & (row.sensing[i] == 0) & (row.connected[i] == 0):
                if row.risk[i] < -1.05 or row.risk[i] > 1.05:
                    if row.hash[i] not in over_risk_hashs:
                        over_risk_hashs.add(row.hash[i])
        sum += len(over_risk_hashs)
    return sum

sums = [calculate_riskRow(df1, False), calculate_riskRow(df2, False), calculate_riskRow(df1, True)]
labels = ["ETSI", "Proposed", "Ideal"]
colors = ["blue", "orange", "green"]

# 危険係数が最大である個数を棒グラフで表示
x = np.arange(3)
fig, ax = plt.subplots()
ax.bar(x, sums, tick_label=labels, color=colors)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Number of collisions")
ax.grid()
plt.title("Number of collisions at each cpm generation method", y=-1.0)
plt.savefig(args[2] + "_riskCoefficient_CollisionCnt.png", dpi=300)
plt.close()
