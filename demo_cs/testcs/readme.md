## control service架构 demo
*** 
> 1. 三个类分别代表了MFR node、PIE node和vs core
> 2. vs core提供接口用于修改和获取数据；
> 3. MFR node和 pie node分别调用这些接口实现通信；
> 4. vs中使用多线程实现，这里为了简便只是用了单线程简单完成了一个数据的打印而已。