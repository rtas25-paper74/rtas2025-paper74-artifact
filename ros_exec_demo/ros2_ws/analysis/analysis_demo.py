#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import matplotlib.pyplot as plt
import json
from dataclasses import dataclass
import os


# In[ ]:


this_file_path = os.path.abspath(__file__)
results_path = os.path.join(os.path.dirname(this_file_path), "../results/preempt_executor.txt")
with open(results_path) as f:
    data = json.load(f)


earliest_time = None
for entry in data:
    if "entry" not in entry or "time" not in entry:
        continue

    time = entry["time"]
    if earliest_time is None or time < earliest_time:
        earliest_time = time

for entry in data:
    if "entry" not in entry or "time" not in entry:
        continue

    time = entry["time"]
    entry["time"] -= earliest_time

# print(data)


# In[ ]:


@dataclass
class Point:
    start: int
    end: int
    name: str

# name -> Point, where point has a start but no end
in_progress = {}

points = []
for entry in data:
    if "entry" not in entry or "time" not in entry:
        continue

    name = entry["entry"]["node"]
    time = entry["time"]
    operation = entry["entry"]["operation"]
    if operation == 'start_work':
        if name in in_progress.keys():
            print(f"Error: {name} already in progress")
        in_progress[name] = Point(time, None, name)
    elif operation == 'end_work':
        if name not in in_progress:
            print(f"Error: {name} not in progress")
        current = in_progress[name]
        current.end = time
        if current.end == current.start:
            current.end += 1
        points.append(current)
        del in_progress[name]
    else:
        print(f"Error: unknown operation {operation}")

if len(in_progress) > 0:
    print(f"Error: {len(in_progress)} in progress at end")

points.sort(key=lambda x: x.start)
print(f"Points: {len(points)}")
print(points)
# make a swimlane plot
fig, ax = plt.subplots()
for i, point in enumerate(points):
    name = point.name
    if name == "mode_switch_timer":
        ypos = 4
        color = 'black'
    elif name == "publisher1":
        ypos = 3
        color = 'red'
    elif name == "publisher2":
        ypos = 1
        color = 'orange'
    elif name == "worker1":
        ypos = 2
        color = 'green'
    elif name == "worker2":
        ypos = 0
        color = 'blue'
    else:
        print(f"Error: unknown name {name}")

    ax.broken_barh([(point.start, point.end - point.start)], (ypos, 0.5), facecolors=color, edgecolor=color)

ax.set_ylim(0, 5)
ax.set_xlim(0, points[-1].end)
ax.set_xlabel('Time')
ax.set_yticks([0.25, 1.25, 2.25, 3.25, 4.25])
ax.set_yticklabels(['worker2', 'publisher2', 'worker1', 'publisher1', 'mode_switch_timer'])

plt.savefig("preempt_executor.pdf")
plt.show()


# In[ ]:




