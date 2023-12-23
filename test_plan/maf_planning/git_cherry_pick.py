import os
import requests
#python3 -m pip install tenacity
from tenacity import retry
from tenacity.stop import stop_after_attempt
from tenacity.wait import wait_exponential

class PncPubPullRequestManager:
    add = 0
    approve = 10
    reject = -10

    def __init__(self, project="maf", repository="maf_planning") -> None:
        self.project = project
        self.repository = repository
        self.auth = (
            "",
            "roz5ielykehjcrhqfq3fideyspdd7dzeufinvipbhku2hako2coa",
        )  # pnc_pub的pat，预计到2024年一月底有效
        self.pnc_pub_id = "363fc650-5dd4-49d9-a156-f4ce76b4b5de"

    @retry(
        reraise=True,
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=2, min=2, max=64),
    )
    def add_code_reviewer(self, pr_id):
        url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/pullRequests/{pr_id}/reviewers?api-version=6.0"
        res = requests.put(url,
                           json={
                               "vote": self.add,
                               "id": self.pnc_pub_id
                           },
                           auth=self.auth)
        if res.status_code != 200:
            if res.status_code < 500:
                return {}
            raise ValueError(
                f"add code reviewer get unexpected status code {res.status_code}, {res.text}"
            )

    @retry(
        reraise=True,
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=2, min=2, max=64),
    )
    def code_review(self, pr_id, vote):
        url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/pullRequests/{pr_id}/reviewers/{self.pnc_pub_id}?api-version=6.0"
        res = requests.put(url, json={"vote": vote}, auth=self.auth)
        if res.status_code != 200:
            raise ValueError(
                f"code_reviewer get unexpected status code {res.status_code}, err: {res.text}"
            )

    @retry(
        reraise=True,
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=2, min=2, max=64),
    )
    def modify_description(self, pr_id, description):
        url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/pullRequests/{pr_id}?api-version=6.0"
        res = requests.patch(url,
                             json={"description": description},
                             auth=self.auth)
        if res.status_code != 200:
            if res.status_code < 500:
                return {}
            raise ValueError(
                f"modify_description get unexpected status code {res.status_code}, err: {res.text}"
            )

    @retry(
        reraise=True,
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=2, min=2, max=64),
    )
    def query_pull_request_by_id(self, pr_id):
        url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/pullRequests/{pr_id}?api-version=6.0"
        res = requests.get(url, auth=self.auth)
        if res.status_code != 200:
            if res.status_code < 500:
                return {}
            raise ValueError(
                f"query pull request[{pr_id}] failed, status_code {res.status_code}, err: {res.text}"
            )
        return res.json()

    @retry(
        reraise=True,
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=2, min=2, max=64),
    )
    def query_commit_id(self, branch):
        url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/commits?api-version=6.0&searchCriteria.itemVersion.version={branch}=branch&searchCriteria.$top=1"
        res = requests.get(url, auth=self.auth)
        if res.status_code != 200:
            if res.status_code < 500:
                return {}
            raise ValueError(
                f"query branch[{branch}] failed, status_code {res.status_code}, err: {res.text}"
            )
        return res.json()

    @retry(
        reraise=True,
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=2, min=2, max=64),
    )
    def query_commits(self, branch):
        commit_list = []
        skip = 0
        limit = 5000
        url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/commits?api-version=6.0&searchCriteria.itemVersion.version={branch}&searchCriteria.itemVersion.versionType=branch&searchCriteria.$top={limit}&searchCriteria.$skip={skip}"
        res = requests.get(url, auth=self.auth)
        if res.status_code != 200:
            raise ValueError(
                f"query branch[{branch}] failed, status_code {res.status_code}, err: {res.text}"
            )
        resJon = res.json()
        count = resJon.get("count", 0)
        while count != 0:
            for va in resJon.get("value", []):
                commit_list.append(va["commitId"])
            skip = skip + count
            url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/commits?api-version=6.0&searchCriteria.itemVersion.version={branch}&searchCriteria.itemVersion.versionType=branch&searchCriteria.$top={limit}&searchCriteria.$skip={skip}"
            res = requests.get(url, auth=self.auth)

            if res.status_code != 200:
                if res.status_code < 500:
                    return []
                raise ValueError(
                    f"query branch[{branch}] failed, status_code {res.status_code}, err: {res.text}"
                )
            resJon = res.json()
            count = resJon.get("count", 0)
        return commit_list

    @retry(
        reraise=True,
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=2, min=2, max=64),
    )
    def query_commits_info(self, branch, limit=100):
        commit_list = []
        skip = 0
        url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/commits?api-version=6.0&searchCriteria.itemVersion.version={branch}&searchCriteria.itemVersion.versionType=branch&searchCriteria.$top={limit}&searchCriteria.$skip={skip}"
        res = requests.get(url, auth=self.auth)
        if res.status_code != 200:
            if res.status_code < 500:
                return []
            raise ValueError(
                f"query branch[{branch}] failed, status_code {res.status_code}, err: {res.text}"
            )
        resJon = res.json()
        resJon.get("count", 0)
        for va in resJon.get("value", []):
            commit = {}
            commit["commit_id"] = va["commitId"]
            commit["author"] = va["author"]
            commit["url"] = va["remoteUrl"]
            commit["comment"] = va["comment"]
            commit_list.append(commit)
        return commit_list

    @retry(
        reraise=True,
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=2, min=2, max=64),
    )
    def query_pullrequest(self, status="active"):
        url = f"https://devops.momenta.works/Momenta/{self.project}/_apis/git/repositories/{self.repository}/pullrequests?searchCriteria.status={status}&api-version=6.0"
        res = requests.get(url, auth=self.auth)
        if res.status_code != 200:
            raise ValueError(
                f"query pullrequest failed, status_code {res.status_code}, err: {res.text}"
            )
        resJson = res.json()
        return resJson.get("value", [])


if __name__ == "__main__":
    source_branch = "junyuan.hu/main/apa_4.0.5_2"
    source_commmit_begin = "ee3d4328ed36511589cc53faa869756aa3ad4078"
    commits_to_pick = []

    git_client = PncPubPullRequestManager()
    commit_list = git_client.query_commits(branch=source_branch)
    for c in commit_list:
        # print("'{}',".format(c))
        commits_to_pick.append(c)
        if c == source_commmit_begin:
            break

    print("{} commmits to pick".format(len(commits_to_pick)))
    if len(commits_to_pick) > 0:
        i = 0
        for c in reversed(commits_to_pick):
            print('[{}/{}] {}'.format(i, len(commits_to_pick), c))
            i += 1

        i = 0
        for c in reversed(commits_to_pick):
            mdi_cmd = 'git cherry-pick {}'.format(c)
            print(mdi_cmd)
            cmd_error = os.system(mdi_cmd)
            if cmd_error == 0:
                print('>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>')
                print('[{}/{}] git cherry-pick {} OK'.format(i, len(commits_to_pick), c))
                print('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
            else:
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                print('[{}/{}] git cherry-pick {} error'.format(i, len(commits_to_pick), c))
                print('??????????????????????????????????????????????????????????????????????')
                break
            
            i += 1
