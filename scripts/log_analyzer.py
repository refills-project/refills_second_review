import json
from collections import defaultdict

import pandas as pd
import numpy as np

with open('log.json', 'r') as f:
    data = json.load(f)

start_angles = set()
goal_angles = set()
for entry in data:
    start_angle = entry['start_angle']
    goal_angle = entry['goal_angle']
    start_angle = str(start_angle)
    goal_angle = str(goal_angle)
    start_angles.add(start_angle)
    goal_angles.add(goal_angle)

table = defaultdict(lambda: pd.DataFrame(index=start_angles, columns=goal_angles))

for entry in data:
    success = entry['success']
    start_angle = str(entry['start_angle'])
    goal_angle = str(entry['goal_angle'])
    table_height = entry['table_height']
    planning_time = entry['planning_time']
    execution_time = entry['execution_time']
    if success:
        table[table_height][start_angle][goal_angle] = planning_time
    else:
        table[table_height][start_angle][goal_angle] = -1

pass