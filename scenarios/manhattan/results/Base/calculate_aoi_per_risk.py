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
def getAoIandRisk(df):
    aoi_vector = 'aoi:vector'
    dist_vector = 'perceptedObjectDistance:vector'
    risk_vector = 'trueRiskRow:vector'
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
    for i in range(11):
        bins.append([])
    for row in new_df.itertuples():
        for i in range(len(row)):
            if row.distance[i] < 300:
                remainder = int(min(abs(row.risk[i]), 1) * 10)
                bins[remainder].append(row.aoi[i])
    risks = []
    r = 0.0
    for i in range(11):
        risks.append(r)
        r += 0.1
    return risks, bins

if __name__ == "__main__":
    args = sys.argv
    # dfnum = len(args) - 1
    # dfs = []
    # データフレームの読み込み
    # for i in range(dfnum):
    df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})
    # dfs.append(df)
    # グラフを描画: RiskごとのAoIの80%パーセンタイル値を描画
    fig, ax = plt.subplots()
    risks, bins = getAoIandRisk(df)
    print(risks, bins)
    aoi_percentile80 = []
    for i in range(len(risks)):
        if len(bins[i]) > 0:  # bins[i]が空でないことを確認
            aoi_percentile80.append(np.percentile(bins[i], 80))
        else:
            aoi_percentile80.append(0)  # 空の場合はNaNを追加
    print(risks, aoi_percentile80)
    ax.plot(risks, aoi_percentile80)
            
    ax.legend()
    ax.set_xlim([0, 1.0])
    ax.set_ylim([0, 1.0])
    ax.tick_params(direction="in")
    ax.set_xticks(np.arange(0, 1.1, 0.1))
    ax.set_yticks(np.arange(0, 1.1, 0.1))
    ax.set_xlabel("Risk")
    ax.set_ylabel("AoI [s]")
    plt.title("80 percentile of AoI at each Risk", y=-1.5)
    plt.savefig(args[1] + "_aoi_per_risk.png", dpi=300)
    plt.show()
    plt.close()
    