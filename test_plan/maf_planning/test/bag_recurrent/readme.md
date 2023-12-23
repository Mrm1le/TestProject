## Parking bag可视化工具

工具用于读取maf2.2.1 1217版本（v1.36和v2.*）的泊车bag。目前提供两种可视化方法，jupyter notebook 网页端运行和可执行脚本运行。
运行需要在ros-dev环境中进行（依赖msg_convert，yaml，和源文件的相对路径等）。

- jupyter notebook 环境依赖
```
sudo apt-get update
sudo apt-get install -y libgoogle-perftools-dev
pip3 install jupyter jupytext pybind11 bokeh yep ipywidgets scipy
```

#### 编译
```
cd [path_to_maf_planning]/test/bag_recurrent/
mkdir build && cd build
cmake ..
make
```
---
可视化脚本文件都位于 bag_recurrent/script中，推荐使用jupyter工具
#### jupyter notebook
- plot_planner.py 脚本用于分析单个bag，需要在脚本中输入bag路径，能够可视化搜索过程，支持调参测试和mkz与ep车型间对比
- plot_perception_diff.py 脚本用于对比同一场景下两个bag之间的感知差异，需要输入2个bag路径，重叠目标车位对比周围感知点线
- ~~plot_planner_fake_search_tree.py [deprecated]~~

#### 可执行脚本
- plot_one_bag.py 利用python可视化绘制结果，部分函数实现在plot_scenario.py中
```
python3 plot_one_bag.py bag_path
```

#### 性能评测脚本 perf_test.py

性能场景分为三类：

- [x] 平行泊车: 已支持
- [x] 垂直泊车：已支持
- [x] 垂直泊车对向有车：已支持

下述代码描述了其能可调的参数，参数含义见注释。其中每个场景的`slot_margin`和`channle_width`都是列表，支持同时赋多个值。

```python
USE_MULTI_PROCESS = True         # use multi process
GEO_TYPE[0] = "14"                # display type: "4"  for rectangle "6" for hexagon， "14" for tetradecagon
tuned_params = {
    "car_params":{
        "rx5":CAR_PARAM_RX5,
        "ep":CAR_PARAM_EP
    },
    "step":0.3,                                                 # 采样间隔
    "scenario":[                                                # 评测场景
        {
            "type": pp.ScenarioType.PARALLEL,                   # 平行泊车
            "channle_width": [5.5],                             # 通道宽度
            "slot_margin": [1.2, 1.4, 1.6, 1.8] ,     # 平行库位长度减去车长后的长度
            "apa_file": APA_PARALLEL
        },
        {
            "type": pp.ScenarioType.VERTICAL_BASE,              # 垂直泊车
            "channle_width": [5.5],                             # 通道宽度
            "slot_margin":[2.6],                                 # 垂直库位的宽度， 这与平行泊车中的参数不一致
            "apa_file": APA_VERTICAL
        },
        {
            "type": pp.ScenarioType.VERTICAL_SINGLE_CAR,        # 垂直泊车对面车伸出库位
            "channle_width": [5.2],                             # 对面车所形成的通道宽度， 原始通道宽度为5.5 
            "slot_margin":[2.6],                                 # 垂直库位的宽度， 这与平行泊车中的参数不一致
            "apa_file": APA_VERTICAL_2ND
        }
    ]
}
```

该脚本运行完成之后，规划结果会以两车对比的形式生成html文件，默认存储路径为`HTML_FILE_PATH = "/home/ros/Downloads/perf_test.html"`.

**新规划器对接**
```c++
// test/bag_recurrent/src/parking_scenario.cpp

// 该脚本调用的规划接口实现
void apaPlan(const msquare::parking::OpenspaceDeciderOutput odo,
             msquare::SbpResult &sbp_result,
             msquare::parking::SearchProcessDebug *sp_debug) {
    
    switch(msquare::HybridAstarConfig::GetInstance()->planning_core){
        case 0:
            msquare::parking::genOpenspacePath(odo, sbp_result, sp_debug);
            break;
        case 1:
            msquare::getRulePlannerRes(odo, sbp_result, sp_debug);
            break;
        default:
            break;
    }

}
// 如果要测试新规划器，需要添加一个新的规划接口函数，并根据'planning_core'参数判断
// odo: 规划输入
// sbp_result: 规划输出
// sp_debug：未使用，默认为nullptr


```

**多进程**

```python
# 多进程参数开关
USE_MULTI_PROCESS = True         # use multi process
```

由于规划参数以单例传入，以及混合A*中使用了全局静态变量，所以使用了多进程而不是多线程。各个进程计算完成之后会写入到文件，最后主进程收集各个文件中的信息返回。进程数为逻辑cpu数目 (pocessor数目)。

多进程会增加生成进程代价和读写文件代价，因此性能评测中点数越多，加速效果越明显。

#### 测量工具

```python
'''
工具路径： plotter.event_tools.MeasureTools
'''
# 1. 本项目内使用
fig = bkp.figure(...)
MeasureTools(fig)  # 创建测量工具

# 2. 项目外使用
# 工具所在脚本：plotter/event_tools.py
# 工具依赖脚本：plotter/basic_layers.py
# 需要拷贝上述两个脚本进行引用
# 在根据figure 创建测量工具
```

## 批量抽帧评测脚本

```python
'''
水平脚本  script/batch_test_parallel.py
垂直脚本  script/batch_test_vertical.py

USAGE:

1. python3 <script> <bag_folder> <html_path>  # 评测并画图
2. python3 <script> <bag_folder>              # 评测不画图

TIPS：
     若提示pickle序列化错误，把bag_folder内序列化文件bag.pickle 删除重新运行
'''

#
```

## jira, cla, clean

- [依赖下载参考](https://momenta.feishu.cn/wiki/wikcnoPlTXuevioxS1XNkKKUyQd#)
- [cla 搜索语法](https://momenta.feishu.cn/wiki/wikcnssx8WEcXTXFV5emwOA51Qb)

脚本位置：test/bag_recurrent/script/batch_plan/bag_loader.py
脚本不依赖本仓库内其他模块，可拷贝出来独立运行



```python
'''
依赖：
    jira:
        pip3 install jira
    pymdi:
        pip3 install --extra-index-url https://artifactory.momenta.works/artifactory/api/pypi/pypi-pl/simple pymdi==0.1.11
    rosbag:

功能：
    jira: download bags with jira ids
    cla: download bags with cla keywords
    clean: reduce bag size with only some topic remaining

运行：
    python3 bag_loader.py [jira|cla|clean]  ...
    - 第一个参数选择类型， 第二个参数是每个类型的辅助参数，长度不定，如果只指定第一个参数，会按照代码内params变量指定的参数进行处理

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

    - 举例：
        - jira:  python3  bag_loader.py jira  yangganggang password  ~/Downloads/apa_bag/0304/ EPL3-6043,EPL3-7471
        - cla:   python3  bag_loader.py cla  ~/Downloads/apa_bag/0304/ 13-2-3,13-2-4
        - clean: python3  bag_loader.py clean  ~/Downloads/apa_bag/0304/

bag类型匹配：
    - bag下载默认按照最小内存占用的下载
    - 高级匹配通过变量query_format指定
    - 举例：
        - 默认格式:
            query_format = {
                'must':[],
                'must_not':[]
            }
            
        - 下载带有'no_cam'的bag:
            query_format = {
                'must':['no_cam'],
                'must_not':[]
            }

        - 下载不带有'no_cam', 不带'trimmed'但是带有'gt'的bag:
            query_format = {
                'must':['gt'],
                'must_not':['no_cam', 'trimmed']
            }
'''
```

## plot_planner_from_json

读取json文件中的序列化odo数据进行规划，调参

json文件中用到的key：

- odo: required, 序列化odo数据
- init_points: optional, 与序列化do数据相关联的多个起点
- slot_type: optional, 指明泊车类型，不指明默认为垂直



