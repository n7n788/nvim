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

# df3 = pd.read_csv(args[3], converters = {
#     'attrvalue': parse_if_number,
#     'binedges': parse_ndarray,
#     'binvalues': parse_ndarray,
#     'vectime': parse_ndarray,
#     'vecvalue': parse_ndarray})

max_cnt = 40
def getAoiCnt(df):
    aoi_vector = 'aoi:vector' 
    aoi_dist_vector = 'perceptedObjectDistance:vector'
    cnt_vector = 'cpmReceivedCount:vector'

    # name='aoi:vector'で、vectimeの値がnullでないものを取り出す 
    aoi = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    # name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
    distances = df[(df["name"] == aoi_dist_vector) & (df["vectime"].notnull())]
    # name='cpmReceivedCount:vector'で、vectimeの値がnullでないものを取り出す
    cnts = df[(df["name"] == cnt_vector) & (df["vectime"].notnull())]

    #データから、"module"と"vecvalue"のカラムのみ取り出す
    aois = aoi[["module", "vecvalue", "vectime"]] #各ノードの他ノードへの認識(1 or 0)
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    aois.rename(columns={"vectime": "time"}, inplace=True)

    distances = distances[["module", "vecvalue"]] #各ノードの他ノードとの距離(m)
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 

    cnts = cnts[["module", "vecvalue"]] # 対象車両のCPMを送信している車両数
    cnts.rename(columns={"vecvalue": "cnt"}, inplace=True)
    
    # 'module'をキーとしてdistancesとperceptionsを結合
    new_df = pd.merge(cnts, aois, on='module', how='inner')
    new_df = pd.merge(new_df, distances, on="module", how='inner')

    bins = {"aoi": [], "cnt": [], "aoi_max": [], "aoi_mean": []}
    for i in range(max_cnt):
        bins["aoi"].append([])
    for row in new_df.itertuples():
        # print(row.module)
        for i in range(len(row.distance)):
            if row.time[i] >= 20 and row.distance[i] < 300:
                remainder = int(row.cnt[i])
                bins["aoi"][remainder].append(row.aoi[i])
    cnt = 0
    for i in range(max_cnt):
        if len(bins["aoi"][i]) != 0:
            bins["aoi_mean"].append(statistics.median(bins["aoi"][i]))
            bins["aoi_max"].append(max(bins["aoi"][i]))
            bins["cnt"].append(cnt)
        cnt += 1
    
    return bins

aoi1 = getAoiCnt(df1)
aoi2 = getAoiCnt(df2)
# aoi3 = getAoiCnt(df3)

aois = [aoi1, aoi2]
labels = ["No Congestion Control", "Congestion Control"]
colors = ["blue", "orange"]

# CPMの送信元×AOIの値の中央値、最大値のグラフを描画
fig, ax = plt.subplots()
for i in range(len(aois)):
    ax.plot(aois[i]["cnt"], aois[i]["aoi_mean"], "-", label=labels[i] + ": median", color=colors[i])
    ax.plot(aois[i]["cnt"], aois[i]["aoi_max"], "--", label=labels[i] + ": max", color=colors[i])
    
ax.set(xlabel='Cpm Source Count', ylabel="Age of Information [s]")
ax.legend(loc="upper right")
ax.tick_params(direction='in')
ax.set_xlim([0, max_cnt + 1])
ax.set_ylim([0, 1.5])
plt.xticks(np.arange(0, max_cnt+5, step=5))
plt.yticks(np.arange(0, 1.5, step=0.1))

plt.savefig(args[1] + "_AOI_cnt_dcc.png", dpi=300)
# plt.show()
plt.close(fig)

