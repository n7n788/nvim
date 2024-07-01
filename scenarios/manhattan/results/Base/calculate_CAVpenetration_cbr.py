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

simulation_time = 25
# パケットエラー率を求める
def getCBR(df):
    global simulation_time
    cbr_vector = 'cbr:vector' 
    cbr_df = df[(df["name"] == cbr_vector) & (df["vectime"].notnull())]

    # CAVのリストを作成
    cpmSource_vector = 'cpmSourceId:vector'
    cpmSourceId = df[(df["name"] == cpmSource_vector) & (df["vectime"].notnull())]
    cavList = cpmSourceId.loc[:, "module"]
    cavList = cavList.values.tolist()
    
    bins = []
    counts = []
    for i in range(simulation_time * 10):
        bins.append(0)
        counts.append(0)  
    for row in cbr_df.itertuples():
        module_name = row.module.split("lteNic")[0] + "middleware.CP"
        if module_name in cavList:
            for i in range(len(row.vectime)):
                if row.vectime[i] >= simulation_time:
                    continue
                # 0.1秒ごとにcbrの平均を取る
                remainder = int(row.vectime[i] * 10)
                bins[remainder] += row.vecvalue[i]
                counts[remainder] += 1

    for i in range(len(bins)):
        if counts[i] == 0:
            bins[i] = 0
        else:
            bins[i] /= counts[i]
    
    times = []
    time = 0
    for i in range(simulation_time * 10):
        times.append(time)
        time += 0.1
    
    return (times, bins)

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
    labels = ["0.2", "0.4", "0.5", "0.6", "0.8"]
    colors = ["b", "y", "g", "r", "m"]
    for i in range(len(labels)):
        times, cbr = getCBR(dfs[i])
        ax.plot(times, cbr, ls="-", label=labels[i], color=colors[i])
            
    ax.set(xlabel='Simulation Time (s)', ylabel="Channel Busy Ratio")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, simulation_time])
    ax.set_ylim([0, 1.0])
    plt.xticks(np.arange(0, simulation_time+5, step=5))
    plt.yticks(np.arange(0, 1.1, step=0.1))
    plt.savefig(args[1] + "_CAVpenetrarion_cbr.png", dpi=300)
    plt.show()
    plt.close(fig)
