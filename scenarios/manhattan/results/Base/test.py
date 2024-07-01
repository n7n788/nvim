# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np
import random as rd
import matplotlib.ticker as ptick  

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

# カラム名のnameが一致するデータを取り出す
def getData(name, column_name):
    data = df[(df["name"]==name) & (df["vectime"].notnull())]
    data = data[["module", "vectime", "vecvalue"]]
    data = data.rename(columns={"vecvalue": column_name})
    return data

# カラム名がnameであるデータをnumpy配列として取り出す
def getNarray(data, name):
    n_data = np.array(data.iloc[0:1].loc[:, name].iloc[-1])
    return n_data

name1 = 'posX:vector'
data = getData(name1, name1)
print(data)
name2 = 'cpmGenerationTime:vector'
data = getData(name2, name2)
print(data)
# AoIのリスクごとの箱ひげ図を描画
# x = [[1, 2, 3, 4], [1.3, 2.3, 3.3, 4.3]]
# fig, ax = plt.subplots()
# labels = ["ETSI", "RRS"]
# label_x = ["Risk0", "Risk1", "Risk2", "Risk3"]
# colors = ["#0000ff", "#ffa500"]
# bins = []
# for i in range(2):
#     bin = []
#     for j in range(4):
#         bin.append(range(1, rd.randint(1, 100)))
#     bins.append(bin)
    
# for i in range(len(bins)):
#     b = [bins[i][0], bins[i][1], bins[i][2], bins[i][3]]
#     bp = ax.boxplot(b, positions=x[i], widths=0.3, whis=1e100,
#                     boxprops=dict(color=colors[i],
#                                   linewidth=1,
#                                   ),
#                     medianprops=dict(color=colors[i], linewidth=1),  # 中央値の線の設定
#                     whiskerprops=dict(color=colors[i], linewidth=1),  # ヒゲの線の設定
#                     capprops=dict(color=colors[i], linewidth=1),  # ヒゲの先端の線の設定
#                     flierprops=dict(markeredgecolor=colors[i], markeredgewidth=1))  # 外れ値の設定

# レジェンドの設定
# legends = []
# colors = ['y-', 'b-']
# plot1, = plt.plot([1, 1], colors[i], linewidth=10)
# plot2, = plt.plot([1, 1], colors[i], linewidth=10)
# # legends.append((plot1, plot2), labels)
# plt.legend((plot1, plot2), labels)
# plot1.set_visible(False)
# plot2.set_visible(False)
    
# ax.set_xticks([1.15, 2.15, 3.15, 4.15])
# ax.set_xticklabels(label_x)
# # ax.bar(x, ny, tick_label=["ETSI", "Proposed", "ETSI", "Proposed", "ETSI", "Proposed", "ETSI", "Proposed"], color=["blue", "orange", "blue", "orange", "blue", "orange", "blue", "orange"])
# ax.legend(loc='upper right')
# ax.tick_params(direction="in")
# ax.set_xlabel("Cpm generation method")
# ax.set_ylabel("AoI (s)")
# # ax.grid()
# ax.yaxis.set_major_formatter(ptick.ScalarFormatter(useMathText=True)) 
# ax.ticklabel_format(style="sci", axis="y", scilimits=(3,3))
# plt.title("AoI at each cpm generation method", y=-1.0)
# plt.savefig("test.png", dpi=300)
# plt.show()
# plt.close()

# np.set_printoptions(threshold=np.inf)
# # pd.set_option("display.max_rows", 1000)
# # print(data)
# # cmodules = []
# for row in data.itertuples():
#     if row.module == "World.node[239].environmentModel": 
#         print(row.vectime)
#     else:
#         print(row.module)
#         print(row.vectime)
        # print(row.vecvalue[0])
#     cmodules.append(row.module)
# cm = sorted(cmodules)
# pd.set_option("display.max_rows", 1000)
# print(cm)
# ndata = getNarray(data, name1)
# np.set_printoptions(threshold=np.inf)
# print(ndata)

# デコード(tbDecoded)とcam受信時(camReceivedGenerationTime)のvectimeと比較
# name1 = 'tbDecoded:vector'
# print(getData(name1, name1))
# name2 = 'camReceivedGenerationTime:vector'
# data1 = getData(name1, name1)
# data2 = getData(name2, name2)
# time1 = getNarray(data1, "vectime")
# data1 = getNarray(data1, name1)
# time2 = getNarray(data2, "vectime")
# data2 = getNarray(data2, name2)

# data = np.stack([time1[0:100], data1[0:100], time2[0:100]], 0).T
# print(data[:, 0:20])
# print(time1[0:20])
# print(data1[0:20])
# print(time2[0:10])
# print(data2[0:10])
# time1 = np.array(data1.iloc[0:1].loc[:, "vectime"].iloc[-1])
# data1 = np.array(data1.iloc[0:1].loc[:, name1].iloc[-1])
# time2 = np.array(data2.iloc[0:1].loc[:, "vectime"].iloc[-1])
# data2 = np.array(data2.iloc[0:1].loc[:, name2].iloc[-1])
# print(time1[0:10])
# print(data1[0:10])
# print(time2[0:10])
# print(data2[0:10])

# name1 = "sensedVehicleId:vector"
# name2 = "sensorName:vector"
# name3 = "sensorMeasureTime:vector"
# time = df[(df["name"]==name1) & (df["vectime"].notnull())]
# time = time[["module", "vectime"]]

# data1 = getData(name1, name1)
# data2 = getData(name2, name2)
# data3 = getData(name3, name3)

# data = pd.merge(time, data1, data2, data3, on='module', how='inner')

# np.set_printoptions(threshold=np.inf)
# np.set_printoptions(threshold=10, edgeitems=20)
# data1.iloc[0:1].loc([:'vectime']).to_csv('test1.csv')
# data.iloc[0:1].to_csv('test.csv')
# print(data1.iloc[0:1].loc[:, "module":"vectime"])
# print(data.iloc[0:1])


# print(data2.iloc[0:1].loc[:, name2].iloc[-1])
# print(data3.iloc[0:1].loc[:, name3].iloc[-1])

# 配列の中身をプリントし、ある時刻のLEMの中身をチェック
# first = 12
# last = 16
# time = np.array(time.iloc[0:1].loc[:, "vectime"].iloc[-1])
# print("time", end=': ')
# print(time[first:last])
# data1 = np.array(data1.iloc[0:1].loc[:, name1].iloc[-1])
# print(name1, end=': ')
# print(data1[first:last])
# data2 = np.array(data2.iloc[0:1].loc[:, name2].iloc[-1])
# print(name2, end=': ')
# print(data2[first:last])
# data3 = np.array(data3.iloc[0:1].loc[:, name3].iloc[-1])
# print(name3, end=': ')
# print(data3[first:last])

# numpy配列をファイルに書き込み
# time = np.array(data1.iloc[0:1].loc[:, "vectime"].iloc[-1])
# np.savetxt('test.csv', time[12:15], delimiter='\t', fmt="%.1e")
# np.savetxt('test.csv', time[12:15], delimiter='\t', fmt="%.1e")
# with open('test.txt') as f:
#   print(f.readlines())

#
# time = np.array(time.iloc[0:1].loc[:, "vectime"].iloc[-1])
# for i in range(len(time)):
