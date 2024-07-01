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

args = sys.argv
df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

# シミュレーションのデータをとる領域
# lower_limit_pos_x = 300
# upper_limit_pos_x = 700
# lower_limit_pos_y = -1000
# upper_limit_pos_y = 1000
# lower_time = 10

def calculate_channel_busy_ratio():
    cbr_vector = "cbr:vector"
    cbr_df = df[(df["name"] == cbr_vector) & (df["vectime"].notnull())]
    cbr_df = cbr_df[["module", "vecvalue", "vectime"]]

    # print(cbr_df)
    
    # cpmの送信元リストを作成
    cpmSource_vector = 'cpmSourceId:vector'
    cpmSourceId = df[(df["name"] == cpmSource_vector) & (df["vectime"].notnull())]
    cpmList = cpmSourceId.loc[:, "module"]
    cpmList = cpmList.values.tolist()
    
    bins = []
    counts = []
    simulation_time = 25
    for i in range(simulation_time * 10):
        bins.append(0)
        counts.append(0)

    for row in cbr_df.itertuples():
        module_name = row.module.split("lteNic")[0] + "middleware.CP"
        if module_name in cpmList:
            # print(row.module)
            for i in range(len(row.vectime)):
                if row.vectime[i] >= simulation_time:
                    continue
                # 0.1秒ごとにcbrの平均を取る
                remainder = int(row.vectime[i] * 10)
                bins[remainder] += row.vecvalue[i]
                counts[remainder] += 1
        # else:
        #     print("No-CAV: ", row.module)
    
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
    
    # print(bins, times)
    fig, ax = plt.subplots()
    ax.plot(times, bins, label="Channel Busy Ratio")
    ax.set(xlabel='Simulation Time (s)', ylabel="CBR")
    ax.legend(loc="lower left")
    ax.tick_params(direction='in')
    ax.set_xlim([0, (max(times) + 1)])
    ax.set_ylim([0, 1.1])
    plt.xticks(np.arange(0, (max(times))+5, step=5))
    plt.yticks(np.arange(0, 1.1, step=0.1))
    plt.savefig(args[1] + "_CBR.png", dpi=300)
    # plt.show()
    plt.close(fig)
    
calculate_channel_busy_ratio()