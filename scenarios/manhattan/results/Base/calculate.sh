#!/bin/sh

python calculate_riskCoefficient_AoI2.py $1 $2
python calculate_riskCoefficient_Cnt2.py $1 $2
python calculate_pdr2.py $1 $2
python calculate_cbr2.py $1 $2
python calculate_cpmContained2_cdf.py $1 $2
