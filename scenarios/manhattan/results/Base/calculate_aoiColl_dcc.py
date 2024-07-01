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

max_aoi = 0.0
def getAoiColl(df):
    global max_aoi
    aoi_vector = 'aoi:vector' 
    aoi_dist_vector = 'perceptedObjectDistance:vector'
    collisionTime_vector = 'collisionTime:vector'

    # name='objectPerception:vector'で、vectimeの値がnullでないものを取り出す 
    aoi = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    # name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
    distances = df[(df["name"] == aoi_dist_vector) & (df["vectime"].notnull())]
    collisionTime = df[(df["name"] == collisionTime_vector) & (df["vectime"].notnull())]
    
    #データから、"module"と"vecvalue"のカラムのみ取り出す
    aois = aoi[["module", "vecvalue", "vectime"]] #各ノードの他ノードへの認識(1 or 0)
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    aois.rename(columns={"vectime": "time"}, inplace=True)

    distances = distances[["module", "vecvalue"]] #各ノードの他ノードとの距離(m)
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 

    collisionTime = collisionTime[["module", "vecvalue"]]
    collisionTime.rename(columns={"vecvalue": "collisionTime"}, inplace=True)
    
    # 'module'をキーとしてdistancesとperceptionsを結合
    new_df = pd.merge(distances, aois, on='module', how='inner')
    new_df = pd.merge(new_df, collisionTime, on='module', how='inner')
    
    # cpmの送信元リストを作成
    cpmSource_vector = 'cpmSourceId:vector'
    cpmSourceId = df[(df["name"] == cpmSource_vector) & (df["vectime"].notnull())]
    cpmList = cpmSourceId.loc[:, "module"]
    cpmList = cpmList.values.tolist()
    
    bins = {"collision_time": [], "aoi": [], "aoi_max": [], "aoi_mean": []}
    max_collisionTime = 24
    collision_div = 1

    for i in range(int(max_collisionTime / collision_div)):
        bins["aoi"].append([])
        
    max_aoi = 0.0
    for row in new_df.itertuples():
        # print(row.module)
        if row.module in cpmList:
            # print(row.module)
            for i in range(len(row.distance)):
                if row.time[i] >= 0 and row.distance[i] < 300:
                    max_aoi = max(max_aoi, row.aoi[i])
                    bins["aoi"][int(row.collisionTime[i] / collision_div)].append(row.aoi[i])
    for i in range(max_collisionTime):
        if len(bins["aoi"][i]) != 0:
            bins["aoi_mean"].append(statistics.median(bins["aoi"][i]))
            bins["aoi_max"].append(max(bins["aoi"][i]))
    
    cT = 0
    for i in range(len(bins["aoi_max"])):
        bins["collision_time"].append(cT)
        cT += 1
        
    return bins

aoi1 = getAoiColl(df1)
aoi2 = getAoiColl(df2)

aois = [aoi1, aoi2]
labels = ["No Congestion Control", "Packet Dropping (3GPP)"]
# labels = ["ETSI dynamic", "Proposed"]
colors = ["blue", "orange"]

# 衝突時間×AoIの値の中央値、最大値のグラフを描画
fig, ax = plt.subplots()
for i in range(len(aois)):
    ax.plot(aois[i]["collision_time"], aois[i]["aoi_mean"], "-", label=labels[i] + ": median", color=colors[i])
    ax.plot(aois[i]["collision_time"], aois[i]["aoi_max"], "--", label=labels[i] + ": max", color=colors[i])
    
ax.set(xlabel='Estimated Collision Time', ylabel="Age of Information [s]")
ax.legend(loc="upper right")
ax.tick_params(direction='in')
ax.set_xlim([0, len(aoi1["collision_time"]) + 1])
ax.set_ylim([0, 1.5])
plt.xticks(np.arange(0, len(aoi1["collision_time"]) + 5, step=5))
plt.yticks(np.arange(0, 1.5, step=0.1))
plt.grid()

plt.savefig(args[1] + "_AOI_collisionTime_dcc.png", dpi=300)
# plt.show()
plt.close(fig)

