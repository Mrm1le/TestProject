import re, sys, os
from pymdi import pymdi
from jira import JIRA
import rosbag

class JiraHhandle:
    def __init__(self, username, password):
        self.jira = self._getJiraHandle(username, password)
        if self.jira is None:
            raise Exception("Failed to init jira account")
    
    def getCLAKeywords(self, jira_ids):
        issue_list = self._getIssueList(jira_ids)
        cla_words = self._getCLAWords(issue_list)
        return cla_words

    def _getJiraHandle(self, username, password):
        self.jira = None
        try:
            self.jira = JIRA("https://jira.momenta.works", basic_auth=(username, password))
        except Exception as exp:
            print("failed to init jira")
            pass
        finally:
            return self.jira
    
    def _getIssueList(self, jira_ids):
        jira_str = ', '.join(jira_ids)
        query = "key in (" + jira_str + ")"
        issue_list = self.jira.search_issues(query, maxResults=500)
        return issue_list

    def _matchKeyWord(self, jira_id, string):
        string_trim = string.strip()
        key_word = string_trim
        if ':' in string_trim:
            key_word = string_trim.split(':')[-1]
        elif len(string_trim) > 5  and string_trim[-4:] == '.bag':
            bag_name = string_trim.split('.')[-2]
            match_res = re.search( r'\d{8}-\d{6}_\d{8}-\d{6}', bag_name)
            if not match_res:
                key_word = string_trim
            else:
                key_word = match_res.group()
        
        if not key_word:
            raise Exception("{}:failed to parse cla from {}".format(jira_id, key_word))
        
        return key_word


    def _getCLAWords(self, issue_list):
        cla_words= []
        for issue in issue_list:
            keyword = self._matchKeyWord(issue.key, issue.fields.customfield_11254)
            # print("get cla keyword: {}".format(keyword))
            cla_words.append((issue.key,keyword))

        return cla_words

class CLADownloader:
    def __init__(self, token, folder, query_format):
        self.token = token
        self.client = pymdi.Client(token)
        self.folder = folder
        self.query_format = query_format
        if not os.path.exists(folder):
            os.makedirs(folder)
    
    
    
    def downBagsWithCLA(self, cla_keywords):
        for cla_keyword in cla_keywords:
            name,md5 = self._search(cla_keyword)
            if name is None:
                print("\033[0;31;40m failed to search{}\033[0m".format(cla_keyword))
                continue

            print("\033[0;32;40m download {}\033[0m".format(name))
            self._download(name, md5)

    def downBagsWithJira(self, jira_ids, jira_handle):
        ids_cla_words = jira_handle.getCLAKeywords(jira_ids)

        for id_cla in ids_cla_words:
            jira_id = id_cla[0]
            cla_keyword = id_cla[1]
            name,md5 = self._search(cla_keyword)
            if name is None:
                print("\033[0;31;40m failed to search{}\033[0m".format(cla_keyword))
                continue
            
            id_name = jira_id + "_" + name
            print("\033[0;32;40m download {}\033[0m".format(id_name))
            self._download(id_name, md5)


    def _getQuery(self, cla_keyword):
        cla_query = {
                "bool": {
                    "must": [
                        {"type": "bag"},
                        {"category": "parking"},
                        {"collect_type": "mpilot-parking"},
                        {'wildcard': {'name': "*{}*".format(cla_keyword)}}
                    ]
                }
            }
            # ,
                    # "must_not": [{'wildcard': {'name': "*no_cam*"}}]
        
        if 'must' in self.query_format:
            for must_word in self.query_format['must']:
                cla_query['bool']['must'].append(
                    {'wildcard': {'name': "*{}*".format(must_word)}}
                )
        if 'must_not' in self.query_format:
            for must_not_word in self.query_format['must_not']:
                if 'must_not' not in cla_query['bool']:
                    cla_query['bool']['must_not'] = []
                cla_query['bool']['must_not'].append(
                    {'wildcard': {'name': "*{}*".format(must_not_word)}}
                )
        
        return cla_query

    def _download(self, name, md5):
        bag_path = os.path.join(self.folder, name)
        print(name)
        with open(bag_path, "wb") as bag_file:
            self.client.download_data(md5, bag_file, threads=10, chunk_size=10 * 1024 * 1024)
    
    def _search(self, cla_keyword):
        result = self.client.query(
            self._getQuery(cla_keyword),
            save_data = ["name", 'size'], 
            limit=50
        )

        datas = result["data"]
        if(result["data"] == None  or len(datas) == 0):
            return (None, None)
        
        # sort by size in reverse order
        reverse_datas = sorted(datas, key = lambda x: x['size'])

        return (reverse_datas[0]['name'], reverse_datas[0]['md5'])

def cleanBag(bag_folder, topic_list):
    bag_list = [val for val in os.listdir(bag_folder) if val[-4:] == ".bag"]
    print("find {} bags".format(len(bag_list)))

    for input_index in range(len(bag_list)):
        path = os.path.join(bag_folder, bag_list[input_index])
        path_new = path[:-4] + "_clean.bag"
        print("process {}".format(bag_list[input_index]))
        with rosbag.Bag(path_new, 'w') as outbag:
            old_bag = rosbag.Bag(path)
            index_count = old_bag.get_message_count(topic_list)
            for topic, msg, t in old_bag.read_messages(topics = topic_list):
                outbag.write(topic, msg, t)

def printHelp():
    print('''
**********************************************
python3.7 %s [jira|cla|clean]
    jira: download bags with jira ids
    cla: download bags with cla keywords
    clean: reduce bag size with only some topic remaining
***********************************************
    '''%(os.path.basename(sys.argv[0])))

def parseArgv():
    if len(sys.argv) <2:
        printHelp()
    
    type_name = sys.argv[1]
    if type_name not in set(['cla', 'jira', 'clean']):
        printHelp()
    pass

    if type_name == 'jira':
        jira_params = params['jira']
        if len(sys.argv) >2:
            jira_params['username'] = sys.argv[2]
        if len(sys.argv) >3:
            jira_params['password'] = sys.argv[3]
        if len(sys.argv) >4:
            jira_params['bag_folder'] = sys.argv[4]
        if len(sys.argv) >5:
            jira_params['jira_ids'] = sys.argv[5].split(',')
        
        jira_handle = JiraHhandle(jira_params['username'], jira_params['password'])
        cla_downloader = CLADownloader(token, jira_params['bag_folder'], query_format)
        cla_downloader.downBagsWithJira(jira_params['jira_ids'], jira_handle)
        return
    
    if type_name == 'cla':
        cla_params = params['cla']
        if len(sys.argv) >2:
            cla_params['bag_folder'] = sys.argv[2]
        if len(sys.argv) >3:
            cla_params['jira_ids'] = sys.argv[3].split(',')
        
        cla_downloader = CLADownloader(token, cla_params['bag_folder'], query_format)
        cla_downloader.downBagsWithCLA(cla_params['jira_ids'])
        return
    
    if type_name == 'clean':
        clean_params = params['clean']
        if len(sys.argv) >2:
            clean_params['bag_folder'] = sys.argv[2]
        if len(sys.argv) >3:
            clean_params['keep_topic_list'] = sys.argv[3].split(',')
        
        cleanBag( clean_params['bag_folder'], clean_params['keep_topic_list'])


if __name__ == "__main__":
    query_format = {
        'must':[],
        'must_not':[]
    }

    token = "ef3fce0e-36b9-40bf-b8d3-e5d64ed5ffbe"   
    params = {
        'jira':{
            'username':'username',
            'password':'pass',
            'bag_folder': "/home/ros/Downloads/bag_loader/",
            'jira_ids': ['EPL3-6043', 'EPL3-7471','EPL3-7401']
        },
        'cla':{
            'bag_folder': "/home/ros/Downloads/bag_loader/",
            'cla_keywords':['13-2-3']
        },
        'clean':{
            'bag_folder': "/home/ros/Downloads/bag_loader/",
            'keep_topic_list': ["/msd/sbp_request", "/perception/fusion/uss_obstacle", "/vehicle/chassis_report"]
        }
    }

    parseArgv()