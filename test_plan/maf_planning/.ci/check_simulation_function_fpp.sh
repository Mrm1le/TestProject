#!/bin/bash

declare -r SCRIPT_FILE="$(readlink -m "${BASH_SOURCE[0]}")"
declare -r SCRIPT_DIR=$(dirname ${SCRIPT_FILE})
cd "${SCRIPT_DIR}"
declare -r PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null)
declare -r offboard_branch="${offboard_branch:-dev}"
declare -r offboard_dir="${offboard_dir:-${PROJECT_ROOT}/planning_offboard}"
declare -r offboard_repo_url="${offboard_repo_url:-https://devops.momenta.works/Momenta/msd-planning/_git/planning_offboard}"
declare -r onboard_branch="${onboard_branch:-$(git rev-parse --abbrev-ref HEAD)}"
declare -r orgin_branch="${onboard_branch:-$(git rev-parse --abbrev-ref HEAD)}"
declare -r start_time=$(date +%s)
declare -r ci_backend_dir="http://10.11.2.47:5000"
echo "start_time:${start_time}"
echo "pr_email:${pr_email}"
echo "pr_id:${pr_id}"
echo "ci_backend_dir:${ci_backend_dir}"
#declare onboard_commit="${onboard_commit:-$(git rev-parse --short=8  --verify origin/${onboard_branch})}"
# 下面的方式会变成获得pr时的commit，但外部无法checkout过去
# declare -r onboard_commit="$(git rev-parse --verify HEAD)"
#declare -r onboard_commit="${onboard_commit:-${SYSTEM_PULLREQUEST_SOURCECOMMITID:-$(git rev-parse --short=8 --verify HEAD)}}"
#declare -r onboard_commit="281d813a"

echo "ci_type: ${ci_type}"

if [ "${ci_type}" = "PullRequest" ]; then
    declare -r trigger_type="PR"
    user_name=${pr_email%@*}
    git config --system user.name shijie.tang
    git config --system user.email shijie.tang@momenta.ai
    declare -r contrast_commit="$(git rev-parse --short=8 --verify HEAD)"
    echo "current branch commit： ${contrast_commit}"
    declare -r contrast_branch="${onboard_branch}_${pr_id}_$(date +%s)"
    git checkout -b ${contrast_branch}
    git branch -v
    git remote -v
    echo "current branch name： ${contrast_branch}"
    git -c http.extraHeader="Authorization: Basic OnZubTZ2cGZ1anBxdGFuaXJxa3pyN2x5dmpvZ2VmanByNTU3ZHV3anA2anZlcG1zZWc3bHE=" -C ${build_directory} push --set-upstream origin ${contrast_branch}
elif [ "${ci_type}" = "Manual" ]; then
    declare -r trigger_type="Manual"
    echo ${pr_email%@*}
    user_name=${pr_email%@*}
elif [ "${ci_type}" = "IndividualCI" ]; then
    declare -r trigger_type="Manual"
    echo ${pr_email%@*}
    user_name=${pr_email%@*}
else
    declare -r trigger_type="Timing"
    user_name='fred.xu'
fi
echo "trigger_type: ${trigger_type}"
if [ "${Onboard_commit}" = "\$(Onboard_commit)" ]; then
    declare -r onboard_commit="${onboard_commit:-$(git rev-parse --short=8 --verify origin/${onboard_branch})}"
    echo "git rev-parse --short=8  --verify origin/${onboard_branch}"
else
    echo "manual set onborad_commit"
    declare -r onboard_commit="${onboard_commit}"
fi
echo "onboard_commit:${onboard_commit}"
#echo "commit:${Onboard_commit}"

function add_robot_reviewer() {
    echo "start adding robot reviewer"
    if [[ ${ci_type} == "PullRequest" ]]; then
        echo "start adding robot reviewer cause pr"
        curl -X POST "${ci_backend_dir}/api/v1/ci/add_reviewer/${ci_tag}"
    fi
    echo "done with adding robot reviewer"
}
# PR时会使np_planning_ci先拒绝PR请求，并删除之前的所有仿真任务
add_robot_reviewer

echo "user_name:${user_name}"
function download_planning_offboard() {
    [ -e "${offboard_dir}" ] && rm -rf "${offboard_dir}"

    bash "${PROJECT_ROOT}/.ci/downloadcode-azure.sh" \
        "${offboard_branch}" \
        "${offboard_dir}" \
        "${offboard_repo_url}"
    if [ "${USE_TEST_BAG_LIST}" = "true" ]; then
        use_test_bag_list
    fi
}

################################################################
# Main Process

# echo ${ci_type}
# if [[ ${ci_type} == 'PullRequest' ]]; then
#   exit 0
# fi
download_planning_offboard

echo "before build push fpp:ci_type: ${ci_type}"
# 等fpp要跑仿真时再打开
cd "${PROJECT_ROOT}"

echo "after build push fpp:ci_type: ${ci_type}"
#export fpp_image=artifactory.momenta.works/docker-pl/fpp_sim:08-18-15-46_1ffbb64d

echo $(ls ${offboard_dir})
cd "${PROJECT_ROOT}"
#docker run -i --rm -v ${offboard_dir}:/opt artifactory.momenta.works/docker-msd-dev/python:npp_sim bash -c "ls /opt && cd /opt/legacy_code/mddp/script/zhouzihan && python3 submit_sim_task.py --scenario HNP_BAD_EVENT_SLOW_LANE_CHANGE --specific_branch ${onboard_commit} --usr_name np_planning_ci --password CI1qaz@WSX --task_name ${ci_type}_${onboard_commit}_ci_sim_test --run_simple_default --ci_tag ${onboard_commit} --control_branch dev_hnp_0425 --uploader '${user_name}' --run17 --submit_only"
#docker run -i --rm -v ${offboard_dir}:/opt artifactory.momenta.works/docker-msd-dev/python:npp_sim bash -c "ls /opt && cd /opt/legacy_code/mddp/script/zhouzihan && python3 submit_sim_task.py --scenario HNP_BAD_EVENT_SLOW_LANE_CHANGE --specific_branch ${onboard_commit} --usr_name tangshijie --password Tsj100132. --task_name ${ci_type}_${onboard_commit}_ci_sim_test --run_simple_default --ci_tag ${onboard_commit} --control_branch dev_hnp_0425 --uploader '${user_name}' --run17 --submit_only"
# docker run -i --rm -v ${offboard_dir}:/opt artifactory.momenta.works/docker-msd-dev/python:npp_sim bash -c "ls /opt && cd /opt/legacy_code/mddp/script/zhouzihan && python3 submit_sim_task.py  --specific_branch $onboard_commit --usr_name cp_planning_ci --password 123456 --task_name ${ci_type}-${onboard_commit}-eftp  --ci_tag $onboard_commit  --uploader '${user_name}' "
#pr触发
echo "do cpp & fpp simulation jobs by ${ci_type} trigger"
docker run -i --rm -v ${offboard_dir}:/opt artifactory.momenta.works/docker-msd-dev/python:ct_sim bash -c "cd /opt/function_ct/ci-script/script &&  \
    python3  submit.py      \
        --fpp_3c_package_config fpp_3c_package_config.json \
        --fpp_3c_package_type cpu \
        --base_commit $onboard_commit \
        --user_name '${user_name}'  \
        --branch ${contrast_branch} \
        --commit ${contrast_commit} \
        --trigger_type ${trigger_type} \
        --pr_id ${pr_id} \
        --task_name ${ci_tag} \
        --maf_version V3.2.1 \
        --product CP \
        --submit_config cpp_min_simulation_config_ddld.json \
        --simulation_scenario_tag simulation_scenario_tag.json \
        --source_branch ${onboard_branch}"
