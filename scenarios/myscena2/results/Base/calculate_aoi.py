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
# upper_limit_pos_x = 500
# lower_limit_pos_x = 250
# upper_limit_pos_y = 866
# lower_limit_pos_y = 433
# lower_time = 10

def calculate_age_of_information():
    aoi_vector = 'aoi:vector' 
    aoi_dist_vector = 'perceptedObjectDistance:vector'
    # pdr_position_x = 'posX:vector'
    # pdr_position_y = 'posY:vector'

    # name='objectPerception:vector'で、vectimeの値がnullでないものを取り出す 
    aoi = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    # name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
    distances = df[(df["name"] == aoi_dist_vector) & (df["vectime"].notnull())]
    # name='posX:vector'で、vectimeがnullでないものをとりだす
    # position_x = df[(df["name"] == pdr_position_x) & (df["vectime"].notnull())]
    # position_y = df[(df["name"] == pdr_position_y) & (df["vectime"].notnull())]

    #データから、"module"と"vecvalue"のカラムのみ取り出す
    aois = aoi[["module", "vecvalue", "vectime"]] #各ノードの他ノードへの認識(1 or 0)
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    aois.rename(columns={"vectime": "time"}, inplace=True)

    distances = distances[["module", "vecvalue"]] #各ノードの他ノードとの距離(m)
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 

    # position_x = position_x[["module", "vectime", "vecvalue"]]
    # position_x.rename(columns={"vecvalue": "position_x"}, inplace=True)
    # position_x.rename(columns={"vectime": "time"}, inplace=True)

    # position_y = position_y[["module", "vectime", "vecvalue"]]
    # position_y.rename(columns={"vecvalue": "position_y"}, inplace=True)
    # position_y.rename(columns={"vectime": "time"}, inplace=True)

    # 'module'をキーとしてdistancesとperceptionsを結合
    new_df = pd.merge(distances, aois, on='module', how='inner')
    # print(new_df["aoi"])
    
    bins = []
    for i in range(10):
        bins.append([])
    max_aoi = 0
    for row in new_df.itertuples():
        # 手動運転車なら無視
        # if re.match("World.node\[[5-9]\d\]", row.module):
        #     continue
        # print(row.module)
        for i in range(len(row.distance)):
            # module_name = row.module.split("environmentModel")[0] + "lteNic.phy"
            # posX_idx = np.searchsorted(position_x[position_x["module"] == module_name].time.iloc[-1], row.time[i], side='left')
            # posX = position_x[position_x["module"] == module_name].position_x.iloc[-1][posX_idx]
            # posY_idx = np.searchsorted(position_y[position_y["module"] == module_name].time.iloc[-1], row.time[i], side='left')
            # posY = position_y[position_y["module"] == module_name].position_y.iloc[-1][posY_idx]
            
            # if posX < lower_limit_pos_x or posX > upper_limit_pos_x or posY < lower_limit_pos_y or posY > upper_limit_pos_y or row.time[i] < lower_time: 
            #     continue
            # print(posX, posY)
            if row.distance[i] < 1000:
                max_aoi = max(max_aoi, row.aoi[i])
                # Ensures that we have everything in 50m chunks
                remainder = int(row.distance[i] // 100)
                bins[remainder].append(row.aoi[i])

    aois = []
    distances = []
    distance = 100
    for dictionary in bins:
        distances.append(str(distance))
        distance += 100

    # 最大AoIが入るようにグラフを描画
    fig, ax = plt.subplots()
    ax.boxplot(bins, whis=1e100)
    ax.set(xlabel='Distance (m)', ylabel="Age of Information [s]")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, len(distances) + 1])
    ax.set_ylim([0, max_aoi + 0.1])
    ax.set_xticklabels(distances)

    plt.savefig(args[1] + "_AOI.png", dpi=300)
    # plt.show()
    plt.close(fig)

    # はずれ値を省いてグラフを描画
    fig, ax = plt.subplots()
    ax.boxplot(bins, whis=1e100)
    ax.set(xlabel='Distance (m)', ylabel="Age of Information [s]")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, len(distances) + 1])
    ax.set_ylim([0, 1.5])
    ax.set_xticklabels(distances)

    plt.savefig(args[1] + "_AOI_removeOutLier.png", dpi=300)
    # plt.show()
    plt.close(fig)

calculate_age_of_information()