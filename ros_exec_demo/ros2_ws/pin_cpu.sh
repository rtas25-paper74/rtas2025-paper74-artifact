#!/bin/bash
mkdir /dev/cpuset
mount -t cgroup -o cpuset cpuset /dev/cpuset
cd /dev/cpuset
mkdir cpu0
echo 0 > cpu0/cpuset.cpus
echo 0 > cpu0/cpuset.mems
echo 1 > cpuset.cpu_exclusive
# echo 0 > cpuset.sched_load_balance
echo 1 > cpu0/cpuset.cpu_exclusive
echo 1 > cpu0/cpuset.mem_exclusive
echo $$ > cpu0/tasks

echo -1 > /proc/sys/kernel/sched_rt_runtime_us
