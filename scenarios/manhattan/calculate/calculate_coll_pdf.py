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


def calculate_collision_time():
    dist_vector = 'perceptedObjectDistance:vector'
    collisionTime_vector = 'collisionTime:vector'
    velocity_vector = 'velocity:vector'
    
    # name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
    distances = df[(df["name"] == dist_vector) & (df["vectime"].notnull())]
    collisionTimes = df[(df["name"] == collisionTime_vector) & (df["vectime"].notnull())]
    velocities = df[(df["name"] == velocity_vector) & (df["vectime"].notnull())]
    
    distances = distances[["module", "vectime", "vecvalue"]] #各ノードの他ノードとの距離(m)
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 
    distances.rename(columns={"vectime": "time"}, inplace=True)
    collisionTimes = collisionTimes[["module", "vecvalue"]]
    collisionTimes.rename(columns={"vecvalue": "collisionTime"}, inplace=True)
    velocities = velocities[["module", "vecvalue"]]
    velocities.rename(columns={"vecvalue": "velocity"}, inplace=True)
    
    # 'module'をキーとしてdistancesとperceptionsを結合
    new_df = pd.merge(distances, collisionTimes, on='module', how='inner')
    new_df = pd.merge(new_df, velocities, on='module', how='inner')
    
    collTs = []
    dist_bins = []
    vel_bins = []
    max_dist = 300
    base_dist = 50
    max_vel = 24
    base_vel = 5
    max_collisionTime = 21
    base_collisionTime = 1
    # for i in range(max_dist / base_dist):
    #     dist_bins.append([])
    # for i in range(max_vel / base_vel):
    #     vel_bins.append([])
    for i in range(max_collisionTime / base_collisionTime):
        dist_bins.append([])
        vel_bins.append([])
    for row in new_df.itertuples():
        for i in range(len(row.distance)):
            if row.time[i] and row.distance[i] < max_dist:
                collTs.append(row.collisionTime[i])
                # dist_bins[int(row.distance[i] / base_dist)].append(row.collisionTime[i])
                # vel_bins[int(row.velocity[i] / base_vel)].append(row.collisionTime[i])
                dist_bins[int(row.collisionTime[i] / base_collisionTime)].append(row.distance[i])
                vel_bins[int(row.collisionTime[i]) / base_collisionTime].append(row.velocity[i])
    cTs = []
    for i in range(0, max_collisionTime, base_collisionTime):
        if i % 5 == 0:
            cTs.append(str(i))
        else:
            cTs.append("")
                       
    # dists = []
    # vels = []
    # for i in range(0, max_dist, base_dist):
    #     if i % 50 == 0:
    #         dists.append(str(i))
    #     else:
    #         dists.append("")
    # for i in range(0, max_vel, base_vel):
    #     if i % 5 == 0:
    #         vels.append(str(i))
    #     else:
    #         vels.append("")
    
    # 衝突予測時間の確率密度分布を描画
    fig, ax = plt.subplots()
    val, base = np.histogram(collTs, bins=21, range=(0.0, 21.0))
    y = val / float(sum(val))
    x = np.convolve(base, np.ones(2) / 2, mode="same")[1:]
    # print("val:", val)
    # print("base: ", base)
    # print("x: ", x)
    # print("y: ", y)
    ax.plot(x, y, label="Collision Time")
    ax.legend()
    ax.set_xlim([0, max_collisionTime + 1])
    ax.set_ylim([0, 1.0])
    ax.tick_params(direction="in")
    ax.set_xticks(np.arange(0, max_collisionTime + 1, 1))
    ax.set_yticks(np.arange(0, 1.0, 0.10))
    ax.set_xlabel("Estimated Collision Time [s]")
    ax.set_ylabel("PDF")
    # ax.grid()
    plt.title("PDF of estimated collision time", y = -1.0)
    plt.savefig(args[1] + "_collisionTime_pdf.png", dpi=300)
    plt.close()
    
    # print(dist_bins)
    # 衝突予測時間x通信距離のグラフ
    fig, ax = plt.subplots()
    ax.boxplot(dist_bins, whis=1e10000)
    ax.set_xlabel("Estimated Collision Time [s]")
    ax.set_ylabel("Distance [m]")
    ax.set_title("Distances at each estimated Collision", y = -0.4)
    # ax.set_xlim([0, max_dist + 50])
    ax.set_ylim([0, max_dist + 1])
    ax.tick_params(direction="in")
    # ax.set_xticks(np.arange(0, max_dist + 1, base_dist))
    ax.set_yticks(np.arange(0, max_dist + 1, base_dist))
    ax.set_xticklabels(cTs)
    # ax.grid()
    plt.savefig(args[1] + "_collisionTime_dist.png", dpi=300)
    plt.close()
    
    # 衝突予測時間x物体の速度のグラフ
    fig, ax = plt.subplots()
    ax.boxplot(vel_bins, whis=1e10000)
    ax.set_xlabel("Estimated Collision Time [s]")
    ax.set_ylabel("velocity [m/s]")
    ax.set_title("Velocities at each estimated collision time [s]", y = -0.4)
    # ax.set_xlim([0, max_vel + 1])
    ax.set_ylim([0, max_vel + 1])
    ax.tick_params(direction="in")
    # ax.set_xticks(np.arange(0, max_vel + 1, base_vel))
    ax.set_yticks(np.arange(0, max_vel + 1, base_vel))
    ax.set_xticklabels(cTs)
    # ax.grid()
    plt.savefig(args[1] + "_collisionTime_velocity.png", dpi=300)
    plt.close()
    
calculate_collision_time()