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

# パケットエラー率を求める
def getPdr(df):
    pdr_vector = 'tbDecoded:vector' 
    pdr_dist_vector = 'txRxDistanceTB:vector'
    distances = df[(df["name"] == pdr_dist_vector) & (df["vectime"].notnull())]
    decoded = df[(df["name"] == pdr_vector) & (df["vectime"].notnull())]
    distances = distances[["module", "vecvalue"]] #各ノードのパケットの送信元との距離
    distances.rename(columns={"vecvalue": "distance"}, inplace=True) 
    decoded = decoded[["module", "vecvalue"]] #各ノードのパケット受信の成功の有無(1 or 0 or -1)
    decoded.rename(columns={"vecvalue": "decode"}, inplace=True)
    new_df = pd.merge(distances, decoded, on='module', how='inner')
    # CAVのリストを作成
    cpmSource_vector = 'cpmSourceId:vector'
    cpmSourceId = df[(df["name"] == cpmSource_vector) & (df["vectime"].notnull())]
    cavList = cpmSourceId.loc[:, "module"]
    cavList = cavList.values.tolist()
    bins = []
    for i in range(100):
        bins.append({"count": 0, "success": 0})
    for row in new_df.itertuples():
        # 手動運転車なら無視
        module_name = row.module.split("lteNic")[0] + "middleware.CP"
        if module_name in cavList:
            for i in range(len(row.distance)):
                if row.distance[i] < 1000:
                    # Ensures that we have everything in 10m chunks
                    remainder = int(row.distance[i] // 10)
                    if row.decode[i] >= 0:
                        # Only count TBs sent i.e. -1 will be ignored in result
                        bins[remainder]["count"] += 1
                        bins[remainder]["success"] += row.decode[i]

    pdrs = []
    distances = []
    distance = 0
    for dictionary in bins:
        if dictionary["count"] == 0:
            pdrs.append(0)
        else:
            # print(dictionary["success"], dictionary["count"])
            pdrs.append((dictionary["success"] / dictionary["count"]))
        distances.append(distance)
        distance += 10
    return (distances, pdrs)

if __name__ == "__main__":
    args = sys.argv
    dfnum = len(args) - 1
    dfs = []
    # データフレームの読み込み
    for i in range(dfnum):
        df = pd.read_csv(args[i + 1], converters = {
        'attrvalue': parse_if_number,
        'binedges': parse_ndarray,
        'binvalues': parse_ndarray,
        'vectime': parse_ndarray,
        'vecvalue': parse_ndarray})
        dfs.append(df)    
    fig, ax = plt.subplots()
    penetrationLabels = ["0.2", "0.4", "0.5", "0.6", "0.8"]
    cpmMethodLabels = ["ETSI", "RRS"]
    colors = ["b", "y", "g", "r", "m"]
    lines = ["-", "--"]
    for i in range(len(cpmMethodLabels)):
        for j in range(len(penetrationLabels)):
            k = len(penetrationLabels) * i + j
            distances, pdrs = getPdr(dfs[k])
            ax.plot(distances, pdrs, ls=lines[i], color=colors[j], label=cpmMethodLabels[i] + ": " + penetrationLabels[j])
            
    ax.set(xlabel='Distance (m)', ylabel="Packet Delivery Ratio")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, 1000])
    ax.set_ylim([0, 1.0])
    plt.xticks(np.arange(0, (max(distances))+100, step=100))
    plt.yticks(np.arange(0, 1.1, step=0.1))
    plt.savefig(args[1] + "_CAVpenetrarion_CPMmethod_pdr.png", dpi=300)
    plt.show()
    plt.close(fig)
