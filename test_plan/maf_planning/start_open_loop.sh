#!/bin/bash


## 变量格式化
# 管道传过来的变量
target_branch=${target_branch#*heads/}
source_branch=${source_branch#*heads/}
main_branch=${branch}
main_commit=${main_commit}
eval_commit=${main_commit}
user_name=${user_name// /.}
user_name=${user_name,,}
task_name=${task_name// /.}
trigger_type=${trigger_type// /.}
build_directory=${build_directory}
pr=${pr_id}

if [[ ! "$pr" =~ ^[0-9]+$ ]];then
    echo "this is not a pull request, 不需要跑评测"
    echo "commit---> ${main_commit}"
    echo "branch--->${main_branch}"
    exit 0
    declare -r contrast_branch="${main_branch}_merge_$(date +%s)"
else
    echo "this is a pull request"
    echo "pr commit--> ${pr_commit_id}"
    echo "pr branch--> ${main_branch}"
    eval_commit=${pr_commit_id}
    declare -r contrast_branch="${main_branch}_pr_trigger_${pr_id}_$(date +%s)"
fi
git config --system user.name  dun.zhou
git config --system user.email dun.zhou@momenta.ai

echo "pr commit ${eval_commit}, target branch ${target_branch}"
## 切换新分支,上传远程仓库：重要区别
git checkout -q -b ${contrast_branch} 2>&1
git branch -v -q 2>&1
B64_PAT='Om82ZGg2b3JyYnFrbWVqZ3V2ajcyaXFkZ3Q1eDR1ZWFkNmNnM3o1eGFxaHozc3ZiZ3g0N2E='
git -c http.extraHeader="Authorization: Basic ${B64_PAT}" -C ${build_directory} push --set-upstream origin ${contrast_branch} 2>&1
echo "-------------------------"
input='{
      "ep_cfg": {},
      "so_cfg": {
            "commit_id":"'"${eval_commit}"'",
            "branch":"'"${contrast_branch}"'",
            "target_branch":"'"${target_branch}"'",
            "source_branch":"'"${source_branch}"'",
            "pr_id":"'"${pr_id}"'"
      },
      "open_loop_cfg": {
            "event_library_ids": [
                  "全量"
                  ],
            "title":"'"${task_name}"'",
            "product_type": "apa",
            "metric_type": 5
      },
      "closed_loop": {},
      "author":"'"${user_name}"'"
}'
tag=`curl -s https://artifactory.momenta.works:443/artifactory/docker-momenta/pnc_pr_gate/ |  grep -a  "<a.*href*" | sed 's/\(.*\)href="\([^"\n]*\)"\(.*\)/\2/g'  | grep dev | tail -n1 | sed  's/\///g'`
docker pull -q artifactory.momenta.works/docker-momenta/pnc_pr_gate:${tag}
docker run --rm artifactory.momenta.works/docker-momenta/pnc_pr_gate:${tag} -s "${trigger_type}" -u "${user_name}" -i "${input}"
git -c http.extraHeader="Authorization: Basic ${B64_PAT}"  push -q  origin --delete ${contrast_branch}  2>&1

