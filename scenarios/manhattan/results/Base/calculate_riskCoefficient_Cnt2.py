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
df1 = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

df2 = pd.read_csv(args[2], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

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
    
    # 危険係数の配列, 危険係数[0.1~1.0]の配列, 危険係数[-1.0~0.0]の配列, 危険係数0の配列, 危険係数[-10.0~-1.1], [1.1~10.0]の配列
    bins = {"risk": [], "accel_risk" : [], "decel_risk": [], "zero_risk": [], "over_risk": []}
    for row in new_df.itertuples():
        for i in range(len(row.risk)):
            if (row.distance[i] < 300) & (row.sensing[i] == 0) & (row.connected[i] == 0):
                bins["risk"].append(row.risk[i])
                if row.risk[i] > 0.05 and row.risk[i] < 1.05:
                    bins["accel_risk"].append(row.risk[i])
                elif row.risk[i] < -0.05 and row.risk[i] > -1.05:
                    bins["decel_risk"].append(row.risk[i])
                elif row.risk[i] < -1.05 or row.risk[i] > 1.05:
                    bins["over_risk"].append(row.risk[i])
                else:
                    bins["zero_risk"].append(0.0)
    return bins

bins = [calculate_riskRow(df1, False), calculate_riskRow(df2, False)]
labels = ["ETSI", "RRS"]
colors = ["blue", "orange"]

risk_y = []
risk_all_y = []
risk_x = []
for i in range(2):
    val, base = np.histogram(bins[i]["risk"], bins=risk_num, range=(min_risk-0.05, max_risk + 0.05)) 
    y = val / float(sum(val))
    all_y = np.add.accumulate(val) / float(sum(val))
    x = np.convolve(base, np.ones(2) / 2, mode="same")[1:]
    risk_y.append(y)
    risk_all_y.append(all_y)
    risk_x.append(x)
# print(risk_x)

# 危険係数の分布を表示    
fig, ax = plt.subplots()
for i in range(2):
    ax.plot(risk_x[i], risk_y[i], label=labels[i], color=colors[i])
ax.legend()
ax.set_xlim(min_risk, max_risk + 0.1)
ax.set_ylim([0, 1.1])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(min_risk, max_risk + 0.1, 2.0))
ax.set_yticks(np.arange(0, 1.1, 0.1))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("PDF")

plt.title("PDF of risk coefficient", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficientCnt2_pdf.png", dpi=300)
plt.close()

# 危険係数の分布のうち, y軸が0.01未満の分布を表示
fig, ax = plt.subplots()
for i in range(2):
    ax.plot(risk_x[i], risk_y[i], label=labels[i], linewidth=1, color=colors[i])
ax.legend()
ax.set_xlim(min_risk, max_risk)
ax.set_ylim([0, 0.01])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(min_risk, max_risk + 0.1, 2.0))
ax.set_yticks(np.arange(0, 0.01, 0.001))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("PDF")

plt.title("PDF of risk coefficient", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficientCnt2_limited_pdf.png", dpi=300)
plt.close()

# 危険係数の累積分布を表示
fig, ax = plt.subplots()
for i in range(2):
    ax.plot(risk_x[i], risk_all_y[i], label=labels[i], linewidth=1, color=colors[i])
ax.legend(loc="lower right")
ax.set_xlim(min_risk, max_risk)
ax.set_ylim([0, 1.0])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(min_risk, max_risk + 0.1, 2.0))
ax.set_yticks(np.arange(0, 1.1, 0.1))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("CDF")

plt.title("CDF of risk coefficient", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficientCnt2_cdf.png", dpi=300)
plt.close()

# 危険係数がオーバーしている個数を棒グラフで表示
x = np.arange(2)
y = []
for i in range(2):
    y.append(len(bins[i]["over_risk"]))
    # print(len(bins[i]["over_risk"]))
fig, ax = plt.subplots()
ax.bar(x, y, tick_label=labels, color=colors)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Count")

ax.yaxis.set_major_formatter(ptick.ScalarFormatter(useMathText=True)) 
ax.ticklabel_format(style="sci", axis="y", scilimits=(3,3))
plt.title("Count of over risk at each cpm generation method", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficientCnt2_over.png", dpi=300)
plt.close()

# 危険係数が0であるCountを棒グラフで表示
x = np.arange(2)
y = []
for i in range(2):
    y.append(len(bins[i]["zero_risk"]))
fig, ax = plt.subplots()
ax.bar(x, y, tick_label=labels, color=colors)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Count")

ax.yaxis.set_major_formatter(ptick.ScalarFormatter(useMathText=True)) 
ax.ticklabel_format(style="sci", axis="y", scilimits=(6,6))
plt.title("Count of risk = 0.0 at each cpm generation method", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficientCnt2_zero.png", dpi=300)
plt.close()

# 危険係数が[0.1~1.0]であるCountを棒グラフで表示
x = np.arange(2)
y = []
for i in range(2):
    y.append(len(bins[i]["accel_risk"]))
fig, ax = plt.subplots()
ax.bar(x, y, tick_label=labels, color=colors)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Count")

ax.yaxis.set_major_formatter(ptick.ScalarFormatter(useMathText=True)) 
ax.ticklabel_format(style="sci", axis="y", scilimits=(3,3))
plt.title("Count of risk = [0.1, 1.0] at each cpm generation method", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficientCnt2_accel.png", dpi=300)
plt.close()

# 危険係数が[-0.1~-1.0]であるCountを棒グラフで表示
x = np.arange(2)
y = []
for i in range(2):
    y.append(len(bins[i]["decel_risk"]))
fig, ax = plt.subplots()
ax.bar(x, y, tick_label=labels, color=colors)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Count")

ax.yaxis.set_major_formatter(ptick.ScalarFormatter(useMathText=True)) 
ax.ticklabel_format(style="sci", axis="y", scilimits=(3,3))
plt.title("Count of risk = [-1.0, -0.1] at each cpm generation method", y=-1.0)
plt.savefig(args[1] + args[2] + "_riskCoefficientCnt2_decel.png", dpi=300)
plt.close()

#　危険係数の分布をプリント
print("危険係数の平均値, 標準偏差, 最小値, 第一四分位数, 中央値, 第三四分位数, 最大値")
for i in range(len(bins)):
    print(labels[i])
    c_array = np.percentile(bins[i]["risk"], q=[0, 25, 50, 75, 100])
    print(str(statistics.mean(bins[i]["risk"])) + ", " + \
          str(statistics.pstdev(bins[i]["risk"])) + ", " + \
          str(c_array[0]) + ", " + \
          str(c_array[1]) + ", " + \
          str(c_array[2]) + ", " +  \
          str(c_array[3]) + ", " + \
          str(c_array[4])    
        )
# Countをプリント
for i in range(len(bins)):
    print(labels[i])
    print(str(len(bins[i]["risk"])))
    print("衝突してしまうケース: " + str(len(bins[i]["over_risk"])))
    print("加速すれば対処可能なケース: " + str(len(bins[i]["accel_risk"])))
    print("減速すれば対処可能なケース: " + str(len(bins[i]["decel_risk"])))
    print("何も必要ないケース: " + str(len(bins[i]["zero_risk"])))
    