# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys

# 文字列を数字、True, False, Noneのいずれかに変換する関数
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

# パケットエラー率を求める
pdr_vector = 'tbDecoded:vector' 
pdr_dist_vector = 'txRxDistanceTB:vector'

# name='tbDecoded:vector'で、vectimeの値がnullでないものを取り出す
distances = df[(df["name"] == pdr_dist_vector) & (df["vectime"].notnull())]
# name='txRxDistanceTB:vector'で、vectimeの値がnullでないものを取り出す 
decoded = df[(df["name"] == pdr_vector) & (df["vectime"].notnull())]

#名前がtbRxDistanceTbのデータから、"module"と"vecvalue"のカラムのみ取り出す
distances = distances[["module", "vecvalue"]] #各ノードのパケットの送信元との距離
distances.rename(columns={"vecvalue": "distance"}, inplace=True) 

decoded = decoded[["module", "vecvalue"]] #各ノードのパケット受信の成功の有無(1 or 0 or -1)
decoded.rename(columns={"vecvalue": "decode"}, inplace=True)
new_df = pd.merge(distances, decoded, on='module', how='inner')

bins = []
for i in range(100):
    bins.append({"count": 1, "success": 0})
for row in new_df.itertuples():
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
    pdrs.append((dictionary["success"] / dictionary["count"] * 100))
    distances.append(distance)
    distance += 10

fig, ax = plt.subplots()
ax.plot(distances, pdrs, label="PDR")
ax.set(xlabel='Distance (m)', ylabel="Packet Delivery Ratio %")
ax.legend(loc="lower left")
ax.tick_params(direction='in')
ax.set_xlim([0, (max(distances) + 1)])
ax.set_ylim([0, 101])
plt.xticks(np.arange(0, (max(distances))+50, step=50))
plt.yticks(np.arange(0, (101), step=10))
plt.show()
plt.savefig(args[1] + "pdr.png", dpi=300)
plt.close(fig)


