#!/bin/bash

echo "para num: $#"
if [ $# != 4 ];
then
    echo "$0 exe .out file_name out_dir"
    exit 1;
fi

gprof $1 $2 > gmon.txt
gprof2dot gmon.txt > $4/$3