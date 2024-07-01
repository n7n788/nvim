# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys

def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s=="false" else s if s else None

def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

args = sys.argv
df = pd.read_csv(args[1], converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime': parse_ndarray,
    'vecvalue': parse_ndarray})

#各ノードがデコードした時刻を表示
pdr_vector = 'tbDecoded:vector'
decoded = df[(df["name"] == pdr_vector) & (df["vectime"].notnull())]
decoded = decoded[["module", "vectime"]] #module名とvectimeカラムを取り出す


print(decoded[0:10])
# test = decoded
# decoded = decoded[["vecvalue"]]
# decoded2 = decoded.iloc[0]
# decoded3 = decoded2[0]
sent_vector = 'sentPacketToLowerLayer:vector(packetBytes)'
sent = df[(df["name"] == sent_vector) & (df["vectime"].notnull())]
sent = sent[["module", "vectime"]]
print(sent[0:10])
