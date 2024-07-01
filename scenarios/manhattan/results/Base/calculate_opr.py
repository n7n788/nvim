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

# シミュレーションのデータをとる領域
upper_limit_pos_x = 500
lower_limit_pos_x = 250
upper_limit_pos_y = 866
lower_limit_pos_y = 433
lower_time = 10

# 物体認識確率を求める関数
def calculate_object_perception_ratio():
    opr_vector = 'objectPerception:vector' 
    opr_dist_vector = 'objectDistance:vector'
    pdr_position_x = 'posX:vector'
    pdr_position_y = 'posY:vector'

    # name='objectPerception:vector'で、vectimeの値がnullでないものを取り出す 
    perceptions = df[(df["name"] == opr_vector) & (df["vectime"].notnull())]
    # name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
    distances = df[(df["name"] == opr_dist_vector) & (df["vectime"].notnull())]
    # name='posX:vector'で、vectimeがnullでないものをとりだす
    position_x = df[(df["name"] == pdr_position_x) & (df["vectime"].notnull())]
    position_y = df[(df["name"] == pdr_position_y) & (df["vectime"].notnull())]

    #データから、"module"と"vecvalue"のカラムのみ取り出す
    perceptions = perceptions[["module", "vecvalue", "vectime"]] #各ノードの他ノードへの認識(1 or 0)
    perceptions.rename(columns={"vecvalue": "perception"}, inplace=True)
    perceptions.rename(columns={"vectime": "time"}, inplace=True)

    distances = distances[["module", "vecvalue"]] #各ノードの他ノードとの距離(m)
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 

    position_x = position_x[["module", "vectime", "vecvalue"]]
    position_x.rename(columns={"vecvalue": "position_x"}, inplace=True)
    position_x.rename(columns={"vectime": "time"}, inplace=True)

    position_y = position_y[["module", "vectime", "vecvalue"]]
    position_y.rename(columns={"vecvalue": "position_y"}, inplace=True)
    position_y.rename(columns={"vectime": "time"}, inplace=True)

    # 'module'をキーとしてdistancesとperceptionsを結合
    new_df = pd.merge(distances, perceptions, on='module', how='inner')

    # print(new_df.info())
    # print(new_df.head())
    # print(position.head())

    bins = []
    for i in range(100):
        bins.append({"count": 0, "success": 0})
    for row in new_df.itertuples():
        # 手動運転車なら無視
        # if re.match("World.node\[[5-9]\d\]", row.module):
        #     continue
        print(row.module)

        for i in range(len(row.distance)):
            module_name = row.module.split("environmentModel")[0] + "lteNic.phy"
            posX_idx = np.searchsorted(position_x[position_x["module"] == module_name].time.iloc[-1], row.time[i], side='left')
            posX = position_x[position_x["module"] == module_name].position_x.iloc[-1][posX_idx]
            posY_idx = np.searchsorted(position_y[position_y["module"] == module_name].time.iloc[-1], row.time[i], side='left')
            posY = position_y[position_y["module"] == module_name].position_y.iloc[-1][posY_idx]

            if posX < lower_limit_pos_x or posX > upper_limit_pos_x or posY < lower_limit_pos_y or posY > upper_limit_pos_y or row.time[i] < lower_time: 
                continue
            if row.distance[i] < 1000:
                # Ensures that we have everything in 10m chunks
                remainder = int(row.distance[i] // 10)
                bins[remainder]["count"] += 1
                bins[remainder]["success"] += row.perception[i]

    oprs = []
    distances = []
    distance = 0
    for dictionary in bins:
        if dictionary["count"] == 0:
            oprs.append(0)
        else:
            oprs.append((dictionary["success"] / dictionary["count"]))
        distances.append(distance)
        distance += 10

    fig, ax = plt.subplots()
    ax.plot(distances, oprs, label="OPR")
    ax.set(xlabel='Distance (m)', ylabel="Object Perception Ratio")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, (max(distances) + 1)])
    ax.set_ylim([0, 1.1])
    plt.xticks(np.arange(0, (max(distances))+100, step=100))
    plt.yticks(np.arange(0, (1.1), step=0.1))
    plt.savefig(args[1] + "_OPR.png", dpi=300)
    plt.show()
    plt.close(fig)

calculate_object_perception_ratio()