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

def calculate_age_of_information():
    aoi_vector = 'aoi:vector' 
    aoi_dist_vector = 'perceptedObjectDistance:vector'
    collisionTime_vector = 'collisionTime:vector'
    riskClass_vector = 'riskClass:vector'
    velocity_vector = 'velocity:vector'
    
    # name='objectPerception:vector'で、vectimeの値がnullでないものを取り出す 
    aoi = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    # name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
    distances = df[(df["name"] == aoi_dist_vector) & (df["vectime"].notnull())]
    collisionTime = df[(df["name"] == collisionTime_vector) & (df["vectime"].notnull())]
    velocity = df[(df["name"] == velocity_vector) & (df["vectime"].notnull())]
    riskClass = df[(df["name"] == riskClass_vector) & (df["vectime"].notnull())]
    #データから、"module"と"vecvalue"のカラムのみ取り出す
    
    aois = aoi[["module", "vecvalue", "vectime"]] 
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    aois.rename(columns={"vectime": "time"}, inplace=True)

    distances = distances[["module", "vecvalue"]] #各ノードの他ノードとの距離(m)
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 
    
    collisionTime = collisionTime[["module", "vecvalue"]]
    collisionTime.rename(columns={"vecvalue": "collisionTime"}, inplace=True)
    
    velocity = velocity[["module", "vecvalue"]]
    velocity.rename(columns={"vecvalue": "velocity"}, inplace=True)
    
    riskClass = riskClass[["module", "vecvalue"]]
    riskClass.rename(columns={"vecvalue": "riskClass"}, inplace=True)
    
    # 'module'をキーとしてdistancesとperceptionsを結合
    new_df = pd.merge(distances, aois, on='module', how='inner')
    new_df = pd.merge(new_df, collisionTime, on='module', how='inner')
    new_df = pd.merge(new_df, riskClass, on='module', how='inner')
    new_df = pd.merge(new_df, velocity, on='module', how='inner')
    # print(new_df)
    
    collisionTime_bins, collisionTimes = [], []
    riskClass_bins, riskClasses = [], []
    velocity_bins, velocities = [], []
    max_collisionTime, max_riskClass, max_velocity = 21, 2, 25
    collisionTime, riskClass, velocity = 1, 0, 1
    for i in range(max_collisionTime):
        collisionTime_bins.append([])
        collisionTimes.append(str(collisionTimes))
        collisionTime += 1
    for i in range(max_riskClass):
        riskClass_bins.append([])
        riskClasses.append(str(riskClass))
        riskClass += 1
    for i in range(max_velocity):
        velocity_bins.append([])
        velocities.append(str(velocity))
        velocity += 1
        
    max_aoi = 0.0
    for row in new_df.itertuples():
        # print(row.module)
        for i in range(len(row.distance)):
            if row.time[i] >= 10 and row.distance[i] < 300:
                max_aoi = max(max_aoi, row.aoi[i])
                collisionTime_bins[int(row.collisionTime[i])].append(row.aoi[i])
                riskClass_bins[int(row.riskClass[i])].append(row.aoi[i])
                velocity_bins[int(row.velocity[i])].append(row.aoi[i])
    # for aoi, time, distance, collisionTime, riskClass, velocity in zip(new_df["aoi"], new_df["time"], new_df["distance"], new_df["collisionTime"], new_df["riskClass"], new_df["velocity"]):
    #     for i in range(len(distance)):
    #         if time[i] >= 10 and distance[i] < 300:
    #             max_aoi = max(max_aoi, aoi[i])
    #             collisionTime_bins[int(collisionTime[i])].append(aoi[i])
    #             riskClass_bins[int(riskClass[i])].append(aoi[i])
    #             velocity_bins[int(velocity[i])].append(aoi[i])
    
    # print(collisionTimes)
    # print(collisionTime_bins)
    # 衝突時間×AoIのグラフを描画
    fig, ax = plt.subplots()
    ax.boxplot(collisionTime_bins, whis=1e100)
    ax.set(xlabel='Collision Time', ylabel="Age of Information [s]")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, len(collisionTimes) + 1])
    ax.set_ylim([0, max_aoi + 0.1])
    ax.set_xticklabels(collisionTimes)
    plt.savefig(args[1] + "_AOI_collisionTime.png", dpi=300)
    # plt.show())
    plt.close(fig)
    
    # 危険度クラス×AoIのグラフを描画
    # fig, ax = plt.subplots()
    # ax.boxplot(riskClass_bins, whis=1e100)
    # ax.set(xlabel='Risk Class', ylabel="Age of Information [s]")
    # ax.legend(loc="lower left")
    # ax.tick_params(direction='in')
    # ax.set_xlim([0, len(riskClass) + 1])
    # ax.set_ylim([0, max_aoi + 0.1])
    # ax.set_xticklabels(riskClass)

    # plt.savefig(args[1] + "_AOI_riskClass.png", dpi=300)
    # # plt.show()
    # plt.close(fig)
    
    # # 速度×AoIのクラスを描画
    # fig, ax = plt.subplots()
    # ax.boxplot(velocity_bins, whis=1e100)
    # ax.set(xlabel='Velocity', ylabel="Age of Information [s]")
    # ax.legend(loc="lower left")
    # ax.tick_params(direction='in')
    # ax.set_xlim([0, len(velocities) + 1])
    # ax.set_ylim([0, max_aoi + 0.1])
    # ax.set_xticklabels(velocities)

    # plt.savefig(args[1] + "_AOI_velocity.png", dpi=300)
    # # plt.show()
    # plt.close(fig)

calculate_age_of_information()