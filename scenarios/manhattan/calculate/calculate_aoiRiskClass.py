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
    riskClass_vector = 'riskClass:vector'

    # name='objectPerception:vector'で、vectimeの値がnullでないものを取り出す 
    aoi = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    # name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
    distances = df[(df["name"] == aoi_dist_vector) & (df["vectime"].notnull())]
    riskClass = df[(df["name"] == riskClass_vector) & (df["vectime"].notnull())]
    
    #データから、"module"と"vecvalue"のカラムのみ取り出す
    aois = aoi[["module", "vecvalue", "vectime"]] #各ノードの他ノードへの認識(1 or 0)
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    aois.rename(columns={"vectime": "time"}, inplace=True)

    distances = distances[["module", "vecvalue"]] #各ノードの他ノードとの距離(m)
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 

    riskClass = riskClass[["module", "vecvalue"]]
    riskClass.rename(columns={"vecvalue": "riskClass"}, inplace=True)
    
    # 'module'をキーとしてdistancesとperceptionsを結合
    new_df = pd.merge(distances, aois, on='module', how='inner')
    new_df = pd.merge(new_df, riskClass, on='module', how='inner')
    
    bins = []
    riskClassNum = 2
    for i in range(riskClassNum):
        bins.append([])
        
    max_aoi = 0.0
    for row in new_df.itertuples():
        # print(row.module)
        for i in range(len(row.distance)):
            if row.time[i] >= 10 and row.distance[i] < 300:
                max_aoi = max(max_aoi, row.aoi[i])
                bins[int(row.riskClass[i])].append(row.aoi[i])
                
    c = 0
    classes = []
    for i in range(len(bins)):
        classes.append(str(c))
        c += 1
        
    fig, ax = plt.subplots()
    ax.boxplot(bins, whis=1e100)
    ax.set(xlabel='Risk Class', ylabel="Age of Information [s]")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, len(classes) + 1])
    ax.set_ylim([0, max_aoi + 0.1])
    ax.set_xticklabels(classes)
    plt.savefig(args[1] + "_AOI_riskClass.png", dpi=300)
    # plt.show())
    plt.close(fig)
    
    # fig = plt.figure()
    # # ax = fig.add_subplot(projection='3d')
    # ax = fig.gca(projection='3d')
    # ax.scatter(distances, reletiveVelocities, aois, s=0.1, color="blue")
    # ax.set_xlabel("Distance (m)")
    # ax.set_ylabel("Reletive Velocity (m/s)")
    # ax.set_zlabel("Age of Information (s)")
    # ax.tick_params(direction='in')
    # ax.set_xlim([0,  310])
    # ax.set_ylim([0, 40])
    # ax.set_zlim([0, 1.1])
    # ax.set_xticklabels(np.arange(0, 310, step=50))
    # ax.set_yticklabels(np.arange(0, 40, step=5))
    # ax.set_zticklabels(np.arange(0, 1.1, step=0.1))
    # plt.savefig(args[1] + "_AOI_reletive.png", dpi=300)
    # plt.close(fig)
    
    # reletiveVelocities = []
    # reletiveVelocity = 0
    # for i in range(8):
    #     reletiveVelocities.append(reletiveVelocity)
    #     reletiveVelocity += 5
        
    # # 50m以内の相対速度×AoIを描画
    # fig, ax = plt.subplots()
    # ax.boxplot(bins50m, whis=1e100)
    # ax.set(xlabel='Reletive Velocity (m/s)', ylabel="Age of Information [s]")
    # ax.legend(loc="lower left")
    # ax.tick_params(direction='in')
    # ax.set_xlim([0, len(reletiveVelocities) + 1])
    # ax.set_ylim([0, 1.1])
    # ax.set_xticklabels(reletiveVelocities)

    # plt.savefig(args[1] + "_AOI_reletive50m.png", dpi=300)
    # plt.close(fig)
    
    # # 100m以内の相対速度×AoIを描画
    # fig, ax = plt.subplots()
    # ax.boxplot(bins100m, whis=1e100)
    # ax.set(xlabel='Reletive Velocity (m/s)', ylabel="Age of Information [s]")
    # ax.legend(loc="lower left")
    # ax.tick_params(direction='in')
    # ax.set_xlim([0, len(reletiveVelocities) + 1])
    # ax.set_ylim([0, 1.1])
    # ax.set_xticklabels(reletiveVelocities)

    # plt.savefig(args[1] + "_AOI_reletive100m.png", dpi=300)
    # plt.close(fig)
    
calculate_age_of_information()