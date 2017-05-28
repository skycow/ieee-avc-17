import numpy as np
import json
import matplotlib.pyplot as plt 
import sys
from scipy.special import erfc

with open('steering_utils/previous_data/ackermann.json', 'rb') as f:
    original = json.load(f)

data = []
for time in original:
    data += original[time]

x = np.sort(data)

x = np.unique(x)

import math
def get_relatived_value(arr, rel_idx):
    forward_bias, idx = math.modf(rel_idx * len(arr))
    if idx+1 == len(arr):
        return arr[-1]
    first_bias = 1 - forward_bias
    v1 = np.array(arr[int(idx)]) * first_bias
    v2 = np.array(arr[int(idx)+1]) * forward_bias
    return (v1 + v2).tolist()

epsilon = 1e-9
def denormalize(value):
    if value >= 1:
        value = 1-epsilon
    if value <= 0:
        value = epsilon
    return get_relatived_value(x, value)