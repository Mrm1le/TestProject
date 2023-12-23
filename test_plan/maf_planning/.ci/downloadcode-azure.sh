#!/usr/bin/env bash

echo "************************ downloadCode.sh begin ************************"
ls -al /root/.ssh
echo "$UID"

gitlabBranch=$1
gitLab_localPath=$2
gitLab_url=$3

function publicExitError(){
    if [[ $1 == 0 ]];then
        echo "$2 success!"
    else
        echo "$2 failed!"
        exit 1
    fi
}

#确定基线代码分支
#gitLab_base_branch=$(echo ${gitlabBranch} | sed "s#refs\\/tags\\/##g")
gitLab_base_branch=${gitlabBranch}

#同步基线代码,与远程仓库保持一致
if [[ $gitLab_localPath == "" ]];then
    publicExitError "1" "gitLab_localPath is empty"
fi
rm -rf $gitLab_localPath
mkdir -p $gitLab_localPath

publicExitError "$?" "mkdir -p $gitLab_localPath"
cd $gitLab_localPath
publicExitError "$?" "cd $gitLab_localPath"

#if [[ $gitLab_url == https* ]];then
#    gitLab_url=$(echo $gitLab_url | sed "s#\\/\\/#\\/\\/$username:$password@#g")
#fi

echo "git clone $gitLab_url $gitLab_localPath -b ${gitLab_base_branch}"

git -c http.extraHeader="Authorization: Basic OmIza3p6ZWVjYmNiZTI2b2JseHN5cWg2bzVlNWk3YWczYXdhNTVjZTU2YmNoNDJyNTU2M2E="  clone --depth 1 $gitLab_url $gitLab_localPath -b ${gitLab_base_branch}
publicExitError "$?" "git clone "

echo "*******display git log************"
git log -n 1
# rm -rf .git
echo "************************ downloadCode.sh end ************************"
