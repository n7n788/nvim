# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
from mpl_toolkits.mplot3d import Axes3D
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


def calculate_age_of_information():
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

    collisionTime_bins = []
    max_collisionTime = 24
    collision_div = 1
    for i in range(int(max_collisionTime / collision_div)):
        collisionTime_bins.append([])
        
    max_aoi = 0.0
    for row in new_df.itertuples():
        # print(row.module)
        if row.module in cpmList:
            # print(row.module)
            for i in range(len(row.distance)):
                if row.time[i] >= 0 and row.distance[i] < 300:
                    max_aoi = max(max_aoi, row.aoi[i])
                    collisionTime_bins[int(row.collisionTime[i] / collision_div)].append(row.aoi[i])
                
    cT = 0
    collisionTimes = []
    for i in range(len(collisionTime_bins)):
        collisionTimes.append(str(cT))
        cT += collision_div
    
    # 衝突予想時間が[0s, 1.0s)の物体のAoIの分布
    val0, base0 = np.histogram(collisionTime_bins[0])
    y0 = val0 / float(sum(val0))
    x0 = np.convolve(base0, np.ones(2) / 2, mode="same")[1:]
    
    # 衝突予想時間が[1.0s, 2.0s)の物体のAoIの分布
    val1, base1 = np.histogram(collisionTime_bins[1])
    y1 = val1 / float(sum(val1))
    x1 = np.convolve(base1, np.ones(2) / 2, mode="same")[1:]
    
     # 衝突予想時間が[20.0s, 21.0s)の物体のAoIの分布
    val20, base20 = np.histogram(collisionTime_bins[20])
    y20 = val20 / float(sum(val20))
    x20 = np.convolve(base20, np.ones(2) / 2, mode="same")[1:]
    
    # 衝突予想時間が[20.0s, 21.0s)の物体のAoIの分布
    val20, base20 = np.histogram(collisionTime_bins[20])
    y20 = val20 / float(sum(val20))
    x20 = np.convolve(base20, np.ones(2) / 2, mode="same")[1:]
    
    fig, ax = plt.subplots()
    ax.plot(x0, y0, label="[0s, 1s)")
    ax.plot(x1, y1, label="[1s, 2s)")
    ax.plot(x20, y20, label="[20s, 21s)")
    ax.legend()
    ax.set_xlim([0, 1.0])
    ax.set_ylim([0, 1.0])
    ax.tick_params(direction='in')
    ax.set_xticks(np.arange(0, 1.0, 0.10))
    ax.set_yticks(np.arange(0, 1.0, 0.10))
    ax.set_xlabel("Age of Information [s]")
    ax.set_ylabel("PDF")
    ax.grid()
    plt.title("PDF of aoi at each estimated collision time", y=-1.0)
    plt.savefig(args[1] + "_AoI_collisionTime_0s_pdf.png", dpi=300)
    plt.close()
    
    fig, ax = plt.subplots()
    ax.plot(x0, y0, label="[0s, 1s)")
    ax.plot(x1, y1, label="[1s, 2s)")
    ax.plot(x20, y20, label="[20s, 21s)")
    ax.legend()
    ax.set_xlim([0, 1.0])
    ax.set_ylim([0, 0.1])
    ax.tick_params(direction='in')
    ax.set_xticks(np.arange(0, 1.0, 0.10))
    ax.set_yticks(np.arange(0, 0.1, 0.01))
    ax.set_xlabel("Age of Information [s]")
    ax.set_ylabel("PDF")
    ax.grid()
    plt.title("PDF of aoi at each estimated collision time", y=-1.0)
    plt.savefig(args[1] + "_AoI_collisionTime_0s_pdf_finely.png", dpi=300)
    plt.close()

calculate_age_of_information()