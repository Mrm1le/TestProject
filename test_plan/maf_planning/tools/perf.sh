#!/bin/bash

script_path=`realpath "${BASH_SOURCE:-$0}"`
script_dir=`dirname ${script_path}`

PERF_CMD=/usr/lib/linux-tools-4.4.0-210/perf

if [ -z $1 ]; then
    output_dir="${script_dir}/perf-output"
else
    mkdir -p $1
    output_dir=`realpath $1`
fi

echo "output_dir: ${output_dir}"
cd ${output_dir}

if ! [ -d "/opt/FlameGraph" ]; then
    git -C /opt clone https://github.com/brendangregg/FlameGraph FlameGraph
fi

if ! [ -x "$(command -v ${PERF_CMD})" ]; then
    echo "failed to find perf"
    exit 1
fi

fpp_pid=$(pidof fpp)
while [ -z "$fpp_pid" ]; do
    fpp_pid=$(pidof fpp)
    sleep 1
    echo "fpp not start"
done
echo "fpp pid: ${fpp_pid}"

sudo ${PERF_CMD} record -a -F 99 -g --call-graph dwarf -p ${fpp_pid} -o perf.data -- sleep 30

sudo ${PERF_CMD} script -i perf.data > perf.script

/opt/FlameGraph/stackcollapse-perf.pl perf.script \
    | /opt/FlameGraph/flamegraph.pl > flamegraph.svg
