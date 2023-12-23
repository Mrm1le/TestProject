#!/bin/bash

set -e

SCRIPT_PATH=`realpath "$0"`
SCRIPT_DIR=`dirname "$SCRIPT_PATH"`

image_tag=${image_tag:-"cpu-maf3.2_v3.2.1-4"}
image_name="artifactory.momenta.works/docker-momenta/np_planning/fpp_run_env:${image_tag}"

docker_file_path=`realpath ${SCRIPT_DIR}/Dockerfile.fpp_run_env_cpu`

docker build -t ${image_name} -f ${docker_file_path} .

