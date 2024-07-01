# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import re
import statistics

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

min_risk = -1.0
max_risk = 1.1
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
    
    risks = df[(df["name"] == risk_vector) & (df["vectime"].notnull())]
    aois = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
    distances = df[(df["name"] == dist_vector) & (df["vectime"].notnull())]
    
    risks = risks[["module", "vecvalue", "vectime"]]
    risks.rename(columns={"vecvalue": "risk"}, inplace=True)
    risks.rename(columns={"vectime": "time"}, inplace=True)
    
    aois = aois[["module", "vecvalue"]]
    aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
    
    distances = distances[["module", "vecvalue"]]
    distances.rename(columns={"vecvalue": "distance"}, inplace=True)
    
    new_df = pd.merge(risks, aois, on="module", how="inner")
    new_df = pd.merge(new_df, distances, on="module", how="inner")
    
    # 危険係数の配列、[0.1~1.0]、[各危険係数のaoiの配列、aoiのx軸となる危険係数の配列
    bins = {"risk": [], "accel_risk" : [], "decel_risk": [], "zero_risk": [], "max_risk": [], "aoi": [], "max_aoi": [], "mean_aoi": [], "risk_x": []}
    for i in range(risk_num):
        bins["aoi"].append([])
    for row in new_df.itertuples():
        for i in range(len(row.risk)):
            if row.distance[i] < 100:
                remainder = 0
                if row.risk[i] > max_risk or row.risk[i] < min_risk:
                    bins["risk"].append(1.1)
                    bins["max_risk"].append(1.1)
                    remainder = int(max_risk * 10 - min_risk * 10)
                    # max_r = max(max_r, 1.1)
                else:
                    bins["risk"].append(row.risk[i])
                    remainder = int(row.risk[i] * 10 - min_risk * 10)
                    if row.risk[i] + 0.01 < 0.0:
                        bins["decel_risk"].append(row.risk[i])
                    elif row.risk[i] - 0.01 > 0.0:
                        bins["accel_risk"].append(row.risk[i])
                    else:
                        bins["zero_risk"].append(0.0)
                        
                bins["aoi"][remainder].append(row.aoi[i])
                max_aoi = max(max_aoi, row.aoi[i])

                # 各車両の各時刻での最大の危険係数と、最小の危険係数を求める
                # if (row.time[i] > last_time + 0.05) 
                # last_time = row.time[i]
                
    r = min_risk            
    for i in range(risk_num):
        bins["max_aoi"].append(statistics.median(bins["aoi"][i]))
        bins["mean_aoi"].append(max(bins["aoi"][i]))
        if i % 10 == 0:
            if i == int(-min_risk * 10):
                bins["risk_x"].append(str(0.0))
            else:
                bins["risk_x"].append(str(r))
        else:
            bins["risk_x"].append("")
        r += 0.1
    # print(bins["risk_x"])
    return bins

bins = [calculate_riskRow(df1, False), calculate_riskRow(df2, False), calculate_riskRow(df2, True)]
# array = range(-10, 12, 1)
# for i in range(len(array)):
#     array[i] *= 0.1
# bins = [{"risk": array}]
labels = ["ETSI", "Proposed", "Ideal"]
risk_y = []
risk_all_y = []
risk_x = []
for i in range(len(bins)):
    val, base = np.histogram(bins[i]["risk"], bins=risk_num, range=(min_risk-0.05, max_risk + 0.05)) 
    y = val / float(sum(val))
    all_y = np.add.accumulate(val) / float(sum(val))
    x = np.convolve(base, np.ones(2) / 2, mode="same")[1:]
    risk_y.append(y)
    risk_all_y.append(all_y)
    risk_x.append(x)

# 危険係数の分布を表示    
fig, ax = plt.subplots()
for i in range(len(risk_x)):
    ax.plot(risk_x[i], risk_y[i], label=labels[i], linewidth=1)
ax.legend()
ax.set_xlim(min_risk, max_risk + 0.1)
ax.set_ylim([0, 1.1])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(min_risk, max_risk + 0.1, 0.5))
ax.set_yticks(np.arange(0, 1.1, 0.1))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("PDF")
ax.grid()
plt.title("PDF of risk coefficient", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_pdf.png", dpi=300)
plt.close()

# y軸が0.01未満の分布を表示
fig, ax = plt.subplots()
for i in range(len(risk_x)):
    ax.plot(risk_x[i], risk_y[i], label=labels[i], linewidth=1)
ax.legend()
ax.set_xlim(min_risk, max_risk)
ax.set_ylim([0, 0.01])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(min_risk, max_risk + 0.1, 0.5))
ax.set_yticks(np.arange(0, 0.01, 0.001))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("PDF")
ax.grid()
plt.title("PDF of risk coefficient", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_limited_pdf.png", dpi=300)
plt.close()

# 累積分布を表示
fig, ax = plt.subplots()
for i in range(len(risk_x)):
    ax.plot(risk_x[i], risk_all_y[i], label=labels[i], linewidth=1)
ax.legend(loc="lower right")
ax.set_xlim(min_risk, max_risk)
ax.set_ylim([0, 1.0])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(min_risk, max_risk + 0.1, 0.5))
ax.set_yticks(np.arange(0, 1.1, 0.1))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("CDF")
ax.grid()
plt.title("CDF of risk coefficient", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_cdf.png", dpi=300)
plt.close()

# 累積分布の[-0.001~0.001]を表示
fig, ax = plt.subplots()
for i in range(len(risk_x)):
    ax.plot(risk_x[i], risk_all_y[i], label=labels[i], linewidth=1)
ax.legend(loc="lower right")
ax.set_xlim(-0.01, 0.0)
ax.set_ylim([0.9, 0.95])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(-0.01, 0.0, 0.001))
ax.set_yticks(np.arange(0.9, 0.95, 0.005))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("CDF")
ax.grid()
plt.title("CDF of risk coefficient", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_limited_cdf.png", dpi=300)
plt.close()

# 危険係数が[0.1~1.0]である個数の分布を表示
risk_y = []
risk_all_y = []
risk_x = []
for i in range(len(bins)):
    val, base = np.histogram(bins[i]["accel_risk"], bins=risk_num, range=(min_risk-0.05, max_risk + 0.05)) 
    y = val / float(sum(val))
    all_y = np.add.accumulate(val) / float(sum(val))
    x = np.convolve(base, np.ones(2) / 2, mode="same")[1:]
    risk_y.append(y)
    risk_all_y.append(all_y)
    risk_x.append(x)
fig, ax = plt.subplots()
for i in range(len(risk_x)):
    ax.plot(risk_x[i], risk_y[i], label=labels[i], linewidth=1)
ax.legend()
ax.set_xlim(0.1, 1.0)
ax.set_ylim([0, 0.2])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(0.1, 1.0, 0.1))
ax.set_yticks(np.arange(0, 0.2, 0.02))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("PDF")
ax.grid()
plt.title("PDF of risk coefficient = [0.1, 1.0]", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_accel_pdf.png", dpi=300)
plt.close()

# 累積分布を表示
fig, ax = plt.subplots()
for i in range(len(risk_x)):
    ax.plot(risk_x[i], risk_all_y[i], label=labels[i], linewidth=1)
ax.legend()
ax.set_xlim(0.1, 1.0)
ax.set_ylim([0, 1.0])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(0.1, 1.0, 0.1))
ax.set_yticks(np.arange(0, 1.0, 0.1))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("CDF")
ax.grid()
plt.title("CDF of risk coefficient = [0.1, 1.0]", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_accel_cdf.png", dpi=300)
plt.close()

# 危険係数が[-1.0~-0.1]である個数の分布を表示
risk_y = []
risk_all_y = []
risk_x = []
for i in range(len(bins)):
    val, base = np.histogram(bins[i]["decel_risk"], bins=risk_num, range=(min_risk-0.05, max_risk + 0.05)) 
    y = val / float(sum(val))
    all_y = np.add.accumulate(val) / float(sum(val))
    x = np.convolve(base, np.ones(2) / 2, mode="same")[1:]
    risk_y.append(y)
    risk_all_y.append(all_y)
    risk_x.append(x)
fig, ax = plt.subplots()
for i in range(len(risk_x)):
    ax.plot(risk_x[i], risk_y[i], label=labels[i], linewidth=1)
ax.legend()
ax.set_xlim(min_risk, -0.1)
ax.set_ylim([0, 0.2])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(-1.0, -0.1, 0.2))
ax.set_yticks(np.arange(0, 0.2, 0.02))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("PDF")
ax.grid()
plt.title("PDF of risk coefficient = [-1.0, -0.1]", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_decel_pdf.png", dpi=300)
plt.close()

# 累積分布を表示
fig, ax = plt.subplots()
for i in range(len(risk_x)):
    ax.plot(risk_x[i], risk_all_y[i], label=labels[i], linewidth=1)
ax.legend()
ax.set_xlim(min_risk, -0.1)
ax.set_ylim([0, 1.0])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(-1.0, -0.1, 0.2))
ax.set_yticks(np.arange(0, 1.0, 0.1))
ax.set_xlabel("Risk Coefficient")
ax.set_ylabel("PDF")
ax.grid()
plt.title("PDF of risk coefficient = [-1.0, -0.1]", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_decel_cdf.png", dpi=300)
plt.close()

# 危険係数が最大である個数を棒グラフで表示
x = np.arange(len(bins))
y = []
for i in range(len(bins)):
    y.append(len(bins[i]["max_risk"]))
fig, ax = plt.subplots()
ax.bar(x, y, label="Risk Coefficient = 1.1", tick_label=labels)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Count")
ax.grid()
plt.title("Count of risk = 1.1 at each cpm generation method", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_max.png", dpi=300)
plt.close()

# 危険係数が0である個数を棒グラフで表示
x = np.arange(len(bins))
y = []
for i in range(len(bins)):
    y.append(len(bins[i]["zero_risk"]))
fig, ax = plt.subplots()
ax.bar(x, y, label="Risk Coefficient = 0.0", tick_label=labels)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Count")
ax.grid()
plt.title("Count of risk = 0,0 at each cpm generation method", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_zero.png", dpi=300)
plt.close()

# 危険係数が[0.1~1.0]である個数を棒グラフで表示
x = np.arange(len(bins))
y = []
for i in range(len(bins)):
    y.append(len(bins[i]["accel_risk"]))
fig, ax = plt.subplots()
ax.bar(x, y, label="Risk Coefficient = [0,1, 1.1]", tick_label=labels)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Count")
ax.grid()
plt.title("Count of risk = [0,1, 1.0] at each cpm generation method", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_accel.png", dpi=300)
plt.close()

# 危険係数が[-0.1~-1.0]である個数を棒グラフで表示
x = np.arange(len(bins))
y = []
for i in range(len(bins)):
    y.append(len(bins[i]["decel_risk"]))
fig, ax = plt.subplots()
ax.bar(x, y, label="Risk Coefficient = [-1.0, -0.1]", tick_label=labels)
ax.legend(loc='lower right')
ax.tick_params(direction="in")
ax.set_xlabel("Cpm generation method")
ax.set_ylabel("Count")
ax.grid()
plt.title("Count of risk = [-1.0, -0.1] at each cpm generation method", y=-1.0)
plt.savefig(args[1] + "_riskCoefficient_decel.png", dpi=300)
plt.close()


# 真の危険係数×AoIの箱ひげ図をそれぞれ表示
fig, ax = plt.subplots()
ax.boxplot(bins[0]["aoi"], whis=1e100)
ax.legend()
ax.set_xlim([0, risk_num + 1])
ax.set_ylim([0, max_aoi + 0.1])
ax.tick_params(direction="in")
ax.set_xticklabels(bins[0]["risk_x"])
ax.set_yticks(np.arange(0, max_aoi + 0.1, 0.5))
ax.set_xlabel("True Risk Coefficient")
ax.set_ylabel("Age of Information [s]")
ax.grid()
plt.title("AoI at each risk coefficient", y=-1.0)
plt.savefig(args[1] + "_aoiRiskCoefficient.png", dpi=300)
plt.close()

fig, ax = plt.subplots()
ax.boxplot(bins[1]["aoi"], whis=1e100)
ax.legend()
ax.set_xlim([0, risk_num + 1])
ax.set_ylim([0, max_aoi + 0.1])
ax.tick_params(direction="in")
ax.set_xticklabels(bins[0]["risk_x"])
ax.set_yticks(np.arange(0, max_aoi + 0.1, 0.5))
ax.set_xlabel("True Risk Coefficient")
ax.set_ylabel("Age of Information [s]")
ax.grid()
plt.title("AoI at each risk coefficient", y=-1.0)
plt.savefig(args[2] + "_aoiRiskCoefficient.png", dpi=300)
plt.close()

# 真の危険係数×AoIの最大値と中央値をそれぞれ表示
x = np.arange(min_risk, max_risk + 0.1, 0.1)
colors = ["blue", "orange"]
fig, ax = plt.subplots()
for i in range(len(bins) - 1):
    ax.plot(x, bins[i]["max_aoi"], "-", label=labels[i] + "_max", color=colors[i])
    ax.plot(x, bins[i]["mean_aoi"], "--", label=labels[i] + "_mean", color=colors[i])
ax.legend()
ax.set_xlim([min_risk, max_risk])
ax.set_ylim([0, max_aoi + 0.1])
ax.tick_params(direction="in")
ax.set_xticks(np.arange(min_risk, max_risk + 0.1, 0.5))
ax.set_yticks(np.arange(0, max_aoi + 0.1, 0.5))
ax.set_xlabel("True Risk Coefficient")
ax.set_ylabel("Age of Information [s]")
ax.grid()
plt.title("AoI at each risk coefficient", y=-1.0)
plt.savefig(args[1] + "_aoiRiskCoefficient_max.png", dpi=300)
plt.close()