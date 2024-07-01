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

max_cnt = 40
def calculate_age_of_information2():
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
    
    # cpmの送信元リストを作成
    cpmSource_vector = 'cpmSourceId:vector'
    cpmSourceId = df[(df["name"] == cpmSource_vector) & (df["vectime"].notnull())]
    cpmList = cpmSourceId.loc[:, "module"]
    cpmList = cpmList.values.tolist()
    
    bins = []
    times = []
    for i in range(max_cnt):
        bins.append([])
        times.append([])
    max_aoi = 0
    for row in new_df.itertuples():
         # 手動運転車なら無視
        if row.module in cpmList:
            # print(row.module)
            for i in range(len(row.distance)):
                if row.time[i] >= 10 and row.distance[i] < 300:
                # if row.distance[i] < 300:
                    # print(row.time[i])
                    max_aoi = max(max_aoi, row.aoi[i])
                    remainder = int(row.distance[i] // 100)
                    bins[int(row.cnt[i])].append(row.aoi[i])
                    times[int(row.cnt[i])].append(row.time[i])

    aois = []
    cnts = []
    cnt = 0
    for dictionary in bins:
        if cnt % 5 == 0:
            cnts.append(str(cnt))
        else:
            cnts.append("")
        cnt += 1

    bins_cnt = []
    for i in range(len(bins)):
        bins_cnt.append(len(bins[i]))
    
    # 最大AoIが入るようにグラフを描画
    fig, ax = plt.subplots()
    ax.boxplot(bins, whis=1e100)
    ax.set(xlabel='Cpm source count', ylabel="Age of Information [s]")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, max_cnt + 1])
    ax.set_ylim([0, max_aoi + 0.1])
    ax.set_xticklabels(cnts)

    plt.savefig(args[1] + "_AOI_cnt.png", dpi=300)
    # plt.show()
    
    plt.close(fig)

    # はずれ値を省いてグラフを描画
    fig, ax = plt.subplots()
    ax.boxplot(bins, whis=1e100)
    ax.set(xlabel='Cpm source count', ylabel="Age of Information [s]")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, max_cnt + 1])
    ax.set_ylim([0, 1.5])
    ax.set_xticklabels(cnts)

    plt.savefig(args[1] + "_AOI_cnt_removeOutLier.png", dpi=300)
    # plt.show()
    
    plt.close(fig)

    # 送信元の時間分布
    fig, ax = plt.subplots()
    ax.boxplot(times, whis=1e100)
    ax.set(xlabel='Cpm source count time', ylabel="cnt")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, max_cnt + 1])
    # ax.set_ylim([0, ])
    ax.set_xticklabels(cnts)

    plt.savefig(args[1] + "_cpmSource_time.png", dpi=300)
    # plt.show()
    
    plt.close(fig)
    
    # cpmの送信元数の分布
    fig, ax = plt.subplots()
    ax.plot(np.arange(0, len(bins), 1), bins_cnt, label="Cpm Source Count")
    ax.set(xlabel='Cpm source count', ylabel="cnt")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, max_cnt + 1])
    # ax.set_ylim([0, ])
    # ax.set_xticklabels(cnts)

    plt.savefig(args[1] + "_cpmSource_cnt.png", dpi=300)
    # plt.show()
    plt.close(fig)

calculate_age_of_information2()