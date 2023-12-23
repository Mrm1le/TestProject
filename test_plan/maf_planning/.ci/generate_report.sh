#!/bin/bash

declare -r SCRIPT_FILE="$(readlink -m "${BASH_SOURCE[0]}")"
declare -r SCRIPT_DIR=$(dirname ${SCRIPT_FILE})
cd "${SCRIPT_DIR}"
declare -r PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null)
declare -r offboard_branch="${offboard_branch:-dev}"
declare -r offboard_dir="${offboard_dir:-${PROJECT_ROOT}/planning_offboard}"
declare -r offboard_repo_url="${offboard_repo_url:-https://devops.momenta.works/Momenta/msd-planning/_git/planning_offboard}"

function download_planning_offboard() {
    [ -e "${offboard_dir}" ] && rm -rf "${offboard_dir}"

    bash "${PROJECT_ROOT}/.ci/downloadcode-azure.sh" \
        "${offboard_branch}" \
        "${offboard_dir}" \
        "${offboard_repo_url}"
}

download_planning_offboard
res=0
docker run -i --rm -v ${offboard_dir}:/opt --env pr_id=${pr_id} artifactory.momenta.works/docker-msd-dev/python:npp_sim bash -c "cd /opt/evaluation/tools/maf_planning_ci && python send_result.py ${ci_tag} ${res} ${scene}"