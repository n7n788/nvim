# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
# import seaboan as sns
import re

#文字列を数字、True, False, Noneのいずれかに変換する関数
def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

# 数値に変換する関数
def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

# データフレームから距離300m未満でRisk=1であるAoIの配列を返す
def getAoI(df):
    aoi_vector = 'aoi:vector'
    dist_vector = 'perceptedObjectDistance:vector'
    risk_vector = 'riskRow:vector'
    aois = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    distances = df[(df["name"] == dist_vector) & (df["vectime"].notnull())]
    risks = df[(df["name"] == risk_vector) & (df["vectime"].notnull())]
    aois = aois[["module", "vecvalue"]]
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    distances = distances[["module", "vecvalue"]]
    distances.rename(columns={"vecvalue": "distance"}, inplace=True)
    risks = risks[["module", "vecvalue"]]
    risks.rename(columns={"vecvalue": "risk"}, inplace=True)
    new_df = pd.merge(aois, distances, on="module", how="inner")
    new_df = pd.merge(new_df, risks, on="module", how="inner")
    bins = []
    for row in new_df.itertuples():
        for i in range(len(row)):
            r = min(abs(row.risk[i]), 1)
            if row.distance[i] < 300 and r > 0.05 and r < 0.95:
                bins.append(row.aoi[i])
    return bins

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
    # AoIのみ取得
    bins = []
    for i in range(dfnum):
        bins.append(getAoI(dfs[i]))
    # グラフを描画: Risk=1であるAoIの累積分布をCAVの普及率ごとに描画
    fig, ax = plt.subplots()
    penetrationLabels = ["0.2", "0.4", "0.5", "0.6", "0.8"]
    cpmMethodLabels = ["ETSI", "RRS"]
    colors = ["b", "y", "g", "r", "m"]
    lines = ["-", "--"]
    for i in range(len(cpmMethodLabels)):
        for j in range(len(penetrationLabels)):
            k = len(penetrationLabels) * i + j
            # print("i: " + i + ", j: " + j + ", k: " + k)
            val, base = np.histogram(bins[k], bins=100) 
            if sum(val) != 0:
                y = np.add.accumulate(val) / float(sum(val))
                x = np.convolve(base, np.ones(2) / 2, mode="same")[1:]
                ax.plot(x, y, ls=lines[i], color=colors[j], label=cpmMethodLabels[i] + ": " + penetrationLabels[j])
            
    ax.legend()
    ax.set_xlim([0, 1.0])
    ax.set_ylim([0, 1.0])
    ax.tick_params(direction="in")
    ax.set_xticks(np.arange(0, 1.1, 0.1))
    ax.set_yticks(np.arange(0, 1.1, 0.1))
    ax.set_xlabel("AoI [s]")
    ax.set_ylabel("CDF")
    plt.title("CDF of AoI at each CAV penetration ratio", y=-1.5)
    plt.savefig(args[1] + "_CAVpenetration_CPMmethod_aoi_with_risk_0_to_1.png", dpi=300)
    plt.show()
    plt.close()
    