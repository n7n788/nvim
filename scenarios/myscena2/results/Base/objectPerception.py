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

opr_vector = 'objectPerception:vector' 
opr_dist_vector = 'objectDistance:vector'

# name='objectPerception:vector'で、vectimeの値がnullでないものを取り出す 
perceptions = df[(df["name"] == opr_vector) & (df["vectime"].notnull())]
# name='objectDistance:vector'で、vectimeの値がnullでないものを取り出す
distances = df[(df["name"] == opr_dist_vector) & (df["vectime"].notnull())]

#データから、"module"と"vecvalue"のカラムのみ取り出す
perceptions = perceptions[["module", "vecvalue"]] #各ノードの他ノードへの認識(1 or 0)
perceptions.rename(columns={"vecvalue": "perception"}, inplace=True)
distances = distances[["module", "vecvalue"]] #各ノードの他ノードとの距離(m)
distances.rename(columns={"vecvalue": "distance"}, inplace=True) 

# print(decoded[0:10])
# print(distances[0:10])
# test = decoded
# decoded = decoded[["vecvalue"]]
# decoded2 = decoded.iloc[0]
# decoded3 = decoded2[0]
# for i in decoded3:
#     if(i != 1 and i != 0):
#         print(i)

# 'module'をキーとしてdistancesとperceptionsを結合
new_df = pd.merge(distances, perceptions, on='module', how='inner')

print(new_df.info())

bins = []
for i in range(100):
    bins.append({"count": 0, "success": 0})
for row in new_df.itertuples():
    for i in range(len(row.distance)):
        if row.distance[i] < 1000:
            # Ensures that we have everything in 10m chunks
            remainder = int(row.distance[i] // 10)
            bins[remainder]["count"] += 1
            bins[remainder]["success"] += row.perception[i]

oprs = []
distances = []
distance = 0
for dictionary in bins:
    if dictionary["count"] == 0:
         oprs.append(0)
    else:
        oprs.append((dictionary["success"] / dictionary["count"] * 100))
    distances.append(distance)
    distance += 10

fig, ax = plt.subplots()
ax.plot(distances, oprs, label="OPR")
ax.set(xlabel='Distance (m)', ylabel="Object Perception Ratio %")
ax.legend(loc="lower left")
ax.tick_params(direction='in')
ax.set_xlim([0, (max(distances) + 1)])
ax.set_ylim([0, 101])
plt.xticks(np.arange(0, (max(distances))+100, step=100))
plt.yticks(np.arange(0, (101), step=10))
plt.show()
plt.savefig("test.png", dpi=300)
plt.close(fig)
