#!/bin/sh

# ETSIのCPM生成手法における、CAVの普及率(0.2, 0.4, 0.5, 0.6, 0.8)のシミュレーション結果
file1_1=test20240425.csv 
file1_2=test20240428.csv 
file1_3=test20240422.csv 
file1_4=test20240614.csv 
file1_5=test20240513.csv

# RRSのCPM生成手法における、CAVの普及率(0.2, 0.4, 0.5, 0.6, 0.8)のシミュレーション結果
file2_1=test20240426.csv
file2_2=test20240501.csv 
file2_3=test20240119.csv 
file2_4=test20240617.csv 
file2_5=test20240519.csv

# 以下をパラレル実行
# CAVの普及率とCPM生成手法ごとのRisk=1のみのAoIの分布を描画
python calculate_CAVpenetration_CPMmethod_aoi_with_risk_1.py \
    $file1_1 $file1_2 $file1_3 $file1_4 $file1_5 $file2_1 $file2_2 $file2_3 $file2_4 $file2_5 &

# CAVの普及率とCPM生成手法ごとのRisk=1を除いたAoIの分布を描画
python calculate_CAVpenetration_CPMmethod_aoi_without_risk_1.py \
    $file1_1 $file1_2 $file1_3 $file1_4 $file1_5 $file2_1 $file2_2 $file2_3 $file2_4 $file2_5 &

wait
