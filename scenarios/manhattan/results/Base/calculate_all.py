# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import calculate_pdr, calculate_per, calculate_cbr, calculate_sci, calculate_aoi, calculate_aoi2, calculate_aoiCollisionTime, calculate_aoiVelocity

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

calculate_cbr.calculate_channel_busy_ratio()
calculate_pdr.calculate_packet_delivary_ratio()
calculate_per.calculate_packet_error_ratio()
calculate_sci.calculate_packet_error_ratio_sci()
calculate_aoi.calculate_age_of_information()
calculate_aoi2.calculate_age_of_information2()
calculate_aoiCollisionTime.calculate_age_of_information()
calculate_aoiVelocity.calculate_age_of_information()
