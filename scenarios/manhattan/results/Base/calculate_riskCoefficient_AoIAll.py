# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import re
import statistics
import matplotlib.ticker as ptick  

#文字列を数字、True, False, Noneのいずれかに変換する関数
def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

# 数値に変換する関数
def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

args = sys.argv
dfs = []
for i in range(len(args) - 1):
    df = pd.read_csv(args[i + 1], converters = {
        'attrvalue': parse_if_number,
        'binedges': parse_ndarray,
        'binvalues': parse_ndarray,
        'vectime': parse_ndarray,
        'vecvalue': parse_ndarray})
    dfs.extend(df)

min_risk = -10.0
max_risk = 10.0
risk_num = int((max_risk - min_risk) * 10) + 1
max_aoi = 0
# trueFlag = Trueの場合、真の危険係数を取得
def calculate_riskRow(df, trueFlag):
    global min_risk, max_risk, risk_num, max_aoi
    risk_vector = 'riskRow:vector'
    if trueFlag:
        risk_vector = 'trueRiskRow:vector'
    aoi_vector = 'aoi:vector'
    dist_vector = 'perceptedObjectDistance:vector'
    sensing_vector = 'sensing:vector'
    connected_vector = 'connected:vector'
    
    risks = df[(df["name"] == risk_vector) & (df["vectime"].notnull())]
    aois = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    distances = df[(df["name"] == dist_vector) & (df["vectime"].notnull())]
    sensings = df[(df["name"] == sensing_vector) & (df["vectime"].notnull())]
    connecteds = df[(df["name"] == connected_vector) & (df["vectime"].notnull())]
    
    risks = risks[["module", "vecvalue", "vectime"]]
    risks.rename(columns={"vecvalue": "risk"}, inplace=True)
    risks.rename(columns={"vectime": "time"}, inplace=True)
    
    aois = aois[["module", "vecvalue"]]
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    
    distances = distances[["module", "vecvalue"]]
    distances.rename(columns={"vecvalue": "distance"}, inplace=True)
    
    sensings = sensings[["module", "vecvalue"]]
    sensings.rename(columns={"vecvalue": "sensing"}, inplace=True)
    
    connecteds = connecteds[["module", "vecvalue"]]
    connecteds.rename(columns={"vecvalue": "connected"}, inplace=True)
    
    new_df = pd.merge(risks, aois, on="module", how="inner")
    new_df = pd.merge(new_df, distances, on="module", how="inner")
    new_df = pd.merge(new_df, sensings, on="module", how="inner")
    new_df = pd.merge(new_df, connecteds, on="module", how="inner")
    
    # 危険係数の配列、危険係数ごとのAoIの配列, 加速が必要な場合のAoIの最大値, 減速が必要な場合のAoIの最大値, 衝突する場合のAoI, 何も必要ない場合のAoI, 対処が必要な場合のAoI
    bins = {"risk": [], "aoi": [], "accel": [], "decel": [], "over": [], "zero": [], "not_zero": []}
    for i in range(risk_num):
        bins["aoi"].append([])
    for row in new_df.itertuples():
        for i in range(len(row.risk)):
            if (row.distance[i] < 300) & (row.sensing[i] == 0) & (row.connected[i] == 0):
                remainder = int(row.risk[i] * 10 - min_risk * 10)
                if len(bins["aoi"]) <= remainder:
                    print(len(bins["aoi"]), remainder)
                bins["aoi"][remainder].append(row.aoi[i])
                max_aoi = max(max_aoi, row.aoi[i])
                
                if row.risk[i] > 0.05 and row.risk[i] < 1.05:
                    bins["accel"].append(row.aoi[i])
                    bins["not_zero"].append(row.aoi[i])
                    # bins["accel"] = max(bins["accel"], row.aoi[i])
                elif row.risk[i] < -0.05 and row.risk[i] > -1.05:
                    bins["decel"].append(row.aoi[i])
                    bins["not_zero"].append(row.aoi[i])
                    # bins["decel"] = max(bins["decel"], row.aoi[i])
                elif row.risk[i] < -1.05 or row.risk[i] > 1.05:
                    bins["over"].append(row.aoi[i])
                    bins["not_zero"].append(row.aoi[i])
                    # bins["over"] = max(bins["over"], row.aoi[i])
                else:
                    bins["zero"].append(row.aoi[i])
                    # bins["zero"] = max(bins["zero"], row.aoi[i])
    r = min_risk            
    for i in range(risk_num):
        bins["risk"].append(r)
        r += 0.1
    return bins

bins = []
for i in range(len(args) - 1):
    bin = calculate_riskRow(dfs[i], True)
bins = [calculate_riskRow(df1, True), calculate_riskRow(df2, True)]
labels = ["0%", "25%", "50%", "75%", "90%", "99%", "100%"]
percentile_q = [0, 25, 50, 75, 90, 99, 100]
vals = [] # 各データセットにおいて, 各危険係数のAoIを分位点ごとに集計
for i in range(len(bins)):
    vals.append([])
    for j in range(len(percentile_q)):
        vals[i].append([])

for i in range(len(bins)):
    for j in range(risk_num):
        if len(bins[i]["aoi"][j]) == 0:
            for k in range(len(percentile_q)):
                vals[i][k].append(0)
        else:
            c_array = np.percentile(bins[i]["aoi"][j], q = percentile_q)
            for k in range(len(percentile_q)):
                vals[i][k].append(c_array[k])

lwindow = -10.0
rwindow = 10.0
step = 2.0
for i in range(len(bins)):
    fig, ax = plt.subplots()
    for j in range(len(percentile_q)):
        ax.plot(bins[i]["risk"], vals[i][j], label=labels[j], linewidth=1)
    ax.legend(bbox_to_anchor=(0.5, 1.075), loc='upper center', ncol=len(percentile_q), borderaxespad=0, fontsize=7.5)
    ax.set_xlim([lwindow, rwindow])
    ax.set_ylim([0, max_aoi + 0.1])
    ax.tick_params(direction="in")
    ax.set_xticks(np.arange(lwindow, rwindow + 0.1, step))
    ax.set_yticks(np.arange(0, max_aoi + 0.1, 1.0))
    ax.set_xlabel("Risk Coefficient")
    ax.set_ylabel("Age of Information [s]")
    # ax.grid()
    plt.title("Percentile of AoI at each risk coefficient", y=-1.0)
    plt.savefig(args[i + 1] + "_riskCoefficient_AoI.png", dpi=300)
    plt.close()

lwindow = -1.0
rwindow = 1.0
step = 0.5
for i in range(len(bins)):
    fig, ax = plt.subplots()
    for j in range(len(percentile_q)):
        ax.plot(bins[i]["risk"], vals[i][j], label=labels[j], linewidth=1)
    ax.legend(bbox_to_anchor=(0.5, 1.075), loc='upper center', ncol=len(percentile_q), borderaxespad=0, fontsize=7.5)
    ax.set_xlim([lwindow, rwindow])
    ax.set_ylim([0, max_aoi + 0.1])
    ax.tick_params(direction="in")
    ax.set_xticks(np.arange(lwindow, rwindow + 0.1, step))
    ax.set_yticks(np.arange(0, max_aoi + 0.1, 1.0))
    ax.set_xlabel("Risk Coefficient")
    ax.set_ylabel("Age of Information [s]")
    # ax.grid()
    plt.title("Percentile of AoI at each risk coefficient", y=-1.0)
    plt.savefig(args[i + 1] + "_riskCoefficient_AoI_limited.png", dpi=300)
    plt.close()


y = []
for i in range(len(bins)):
    print(args[i + 1])
    print("     aoiの平均値, 標準偏差, 最小値, 第一四分位数, 中央値, 第三四分位数, 最大値")
    c_array = np.percentile(bins[i]["accel"], q=[0, 25, 50, 75, 100])
    print("     加速が必要な場合: " + str(statistics.mean(bins[i]["accel"])) + ", " + \
                                      str(statistics.pstdev(bins[i]["accel"])) + ", " + \
                                      str(c_array[0]) + ", " + \
                                      str(c_array[1]) + ", " + \
                                      str(c_array[2]) + ", " +  \
                                      str(c_array[3]) + ", " + \
                                      str(c_array[4])    
                                      )
    c2 = c_array[4]
    c_array = np.percentile(bins[i]["decel"], q=[0, 25, 50, 75, 100])
    print("     減速が必要な場合: " + str(statistics.mean(bins[i]["decel"])) + ", " + \
                                      str(statistics.pstdev(bins[i]["decel"])) + ", " + \
                                      str(c_array[0]) + ", " + \
                                      str(c_array[1]) + ", " + \
                                      str(c_array[2]) + ", " +  \
                                      str(c_array[3]) + ", " + \
                                      str(c_array[4])    
                                      )
    c1 = c_array[4]
    c_array = np.percentile(bins[i]["over"], q=[0, 25, 50, 75, 100])
    print("     衝突してしまう場合: " + str(statistics.mean(bins[i]["over"])) + ", " + \
                                      str(statistics.pstdev(bins[i]["over"])) + ", " + \
                                      str(c_array[0]) + ", " + \
                                      str(c_array[1]) + ", " + \
                                      str(c_array[2]) + ", " +  \
                                      str(c_array[3]) + ", " + \
                                      str(c_array[4])    
                                      )
    c3 = c_array[4]
    c_array = np.percentile(bins[i]["zero"], q=[0, 25, 50, 75, 100])
    print("     何も必要ない場合: " + str(statistics.mean(bins[i]["zero"])) + ", " + \
                                      str(statistics.pstdev(bins[i]["zero"])) + ", " + \
                                      str(c_array[0]) + ", " + \
                                      str(c_array[1]) + ", " + \
                                      str(c_array[2]) + ", " +  \
                                      str(c_array[3]) + ", " + \
                                      str(c_array[4])    
                                      )
    c0 = c_array[4]
    c_array = np.percentile(bins[i]["not_zero"], q=[0, 25, 50, 75, 100])
    print("     対処が必要な場合: " + str(statistics.mean(bins[i]["not_zero"])) + ", " + \
                                      str(statistics.pstdev(bins[i]["not_zero"])) + ", " + \
                                      str(c_array[0]) + ", " + \
                                      str(c_array[1]) + ", " + \
                                      str(c_array[2]) + ", " +  \
                                      str(c_array[3]) + ", " + \
                                      str(c_array[4])    
                                      )
    # print("     減速が必要な場合: " + str(max(bins[i]["decel"])) + " s")
    # print("     衝突してしまう場合: " + str(max(bins[i]["over"])) + " s")
    # print("     何も必要ない場合: " + str(max(bins[i]["zero"])) + " s")
    # y[i].append(c0)
    # y[i].append(c1)
    # y[i].append(c2)
    # y[i].append(c3)
    y.append([c0, c1, c2, c3])

# ny = []
# for i in range(len(y[0])):
#     for j in range(2):
#         ny.append(y[j][i])
# print(ny)
# AoIの最大値の棒グラフを表示
x1 = [1, 2, 3, 4]
x2 = [1.3, 2.3, 3.3, 4.3]
fig, ax = plt.subplots()
label_x = ["Risk0", "Risk1", "Risk2", "Risk3"]
ax.bar(x1, y[0], width=0.3, label="ETSI", align="center", color="blue")
ax.bar(x2, y[1], width=0.3, label="RRS", align="center", color="orange")
ax.set_xticks([1.15, 2.15, 3.15, 4.15])
ax.set_xticklabels(label_x)
# ax.bar(x, ny, tick_label=["ETSI", "Proposed", "ETSI", "Proposed", "ETSI", "Proposed", "ETSI", "Proposed"], color=["blue", "orange", "blue", "orange", "blue", "orange", "blue", "orange"])
ax.legend(loc='upper right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("AoI (s)")
# ax.grid()
# ax.yaxis.set_major_formatter(ptick.ScalarFormatter(useMathText=True)) 
# ax.ticklabel_format(style="sci", axis="y", scilimits=(3,3))s
plt.title("AoI max at each cpm generation method", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficient_AoIMax.png", dpi=300)
plt.close()

# AoIのリスクごとの箱ひげ図を描画
x = [[1, 2, 3, 4], [1.3, 2.3, 3.3, 4.3]]
fig, ax = plt.subplots()
label = ["ETSI", "RRS"]
label_x = ["Risk0", "Risk1", "Risk2", "Risk3"]
colors = ["#0000ff", "#ffa500"]

for i in range(len(bins)):
    b = [bins[i]["zero"], bins[i]["decel"], bins[i]["accel"], bins[i]["over"]]
    bp = ax.boxplot(b, positions=x[i], widths=0.3, whis=1e100,
                    boxprops=dict(color=colors[i],
                                  linewidth=1,
                                  ),
                    medianprops=dict(color=colors[i], linewidth=1),  # 中央値の線の設定
                    whiskerprops=dict(color=colors[i], linewidth=1),  # ヒゲの線の設定
                    capprops=dict(color=colors[i], linewidth=1),  # ヒゲの先端の線の設定
                    flierprops=dict(markeredgecolor=colors[i], markeredgewidth=1))  # 外れ値の設定
    
ax.set_xticks([1.15, 2.15, 3.15, 4.15])
ax.set_xticklabels(label_x)
ax.legend(loc='upper right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("AoI (s)")
# ax.grid()
# ax.yaxis.set_major_formatter(ptick.ScalarFormatter(useMathText=True)) 
# ax.ticklabel_format(style="sci", axis="y", scilimits=(3,3))
plt.title("AoI at each cpm generation method", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficient_AoIbox.png", dpi=300)
plt.show()
plt.close()