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

aoi_vector = 'aoi:vector' 
aoi_dist_vector = 'perceptedObjectDistance:vector'

# name='objectPerception:vector'で、vectimeの値がnullでないものを取り出す 
aoi = df[(df["name"] == aoi_vector) & (df["vectime"].notnull())]
# name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
distances = df[(df["name"] == aoi_dist_vector) & (df["vectime"].notnull())]

#データから、"module"と"vecvalue"のカラムのみ取り出す
aois = aoi[["module", "vecvalue"]] #各ノードの他ノードへの認識(1 or 0)
aois.rename(columns={"vecvalue": "aoi"}, inplace=True)
distances = distances[["module", "vecvalue"]] #各ノードの他ノードとの距離(m)
distances.rename(columns={"vecvalue": "distance"}, inplace=True) 

# 'module'をキーとしてdistancesとperceptionsを結合
new_df = pd.merge(distances, aois, on='module', how='inner')

# print(new_df.info())
# print(new_df)

max_aoi = 0
bins = []
for i in range(10):
    bins.append([])
for row in new_df.itertuples():
    for i in range(len(row.distance)):
        max_aoi = max(max_aoi, row.aoi[i])
        if row.distance[i] < 1000:
            # Ensures that we have everything in 100m chunks
            remainder = int(row.distance[i] // 100)
            bins[remainder].append(row.aoi[i])
# print(bins)
# bins = [[0, 1, 2, 3, 4, 5], [2, 2.1, 2.2, 2.3, 2.4, 2.5], [3, 3.1, 3.1, 3.2, 3.2, 3.3, 3.4, 3.4, 3.5]]
aois = []
distances = []
distance = 100
for dictionary in bins:
    distances.append(str(distance))
    distance += 100

fig, ax = plt.subplots()
ax.boxplot(bins)
ax.set(xlabel='Distance (m)', ylabel="Age of Information")
ax.legend(loc="lower left")
ax.tick_params(direction='in')
ax.set_xlim([0, len(distances) + 1])
ax.set_ylim([0, max_aoi + 0.1])
ax.set_xticklabels(distances)

# plt.yticks(np.arange(0, (max_aoi + 1), step=100))
plt.show()
plt.savefig("test.png", dpi=300)
plt.close(fig)
