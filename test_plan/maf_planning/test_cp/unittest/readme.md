## run

1. sudo apt install lcov    // 安装 lcov依赖
2. 编译：

```
maf_planning 模块下面

mkdir build
cd build
cmake .. 
make
make parking_unittest_coverage
```

3. 查看：在build/test/unittest/parking_unittest_coverage 有html文件生成

## default workspace

```json
// maf_planning/test/unittest
├── CMakeLists.txt
├── bin                 
│   ├── parking_test
├── include                 // 公共头文件
│   ├── i1.h
│   └── i2.h
├── resources               // 辅助数据文件
│   └── apa_decider2021-11-08_20-58-18302764.yaml
└── src                     // 测试脚本源文件 按模块存放
    ├── main.cpp            // 测试脚本入口
    ├── module1             // 模块1
    │   ├── a1.cpp
    │   ├── b1.cpp
    └── module2             // 模块2
        ├── a2.cpp
        └── b2.cpp
    ...                     // 根据需要添加模块

```

## risk assessment

the cmake variable has been modified: the detail is demonstrated in this commit

## 踩坑记录

1. cannot write to .total
```yaml
有可能是lcov 1.12 版本内部错误，更新到1.13问题解决

更新方式：
    1. 下载lcov1.13 .deb 格式安装包：https://ubuntu.pkgs.org/18.04/ubuntu-universe-arm64/lcov_1.13-3_all.deb.html
    2. 安装deb包完成安装
```

