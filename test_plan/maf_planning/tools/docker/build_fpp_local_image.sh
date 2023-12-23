#!/bin/bash

set -e

SCRIPT_PATH=`realpath "$0"`
SCRIPT_DIR=`dirname "$SCRIPT_PATH"`

image_tag=${image_tag:-"maf3.2.1-5"}
image_name="artifactory.momenta.works/docker-momenta/np_planning/fpp_local:${image_tag}"

docker_file_path=`realpath ${SCRIPT_DIR}/Dockerfile.fpp_local`

docker build -t ${image_name} -f ${docker_file_path} .

