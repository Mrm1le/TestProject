# 进程池使用示例
# Author: Fred Xu
# Date: 2022/03/21
# 由于python全局解释器锁的缘故，python的多线程是“伪多线程(无法真正在多个CPU核上运行)”
# 故而我们使用进程池来代替线程池方案。
from multiprocessing import Pool

# 引入异常处理模块
import traceback

# 假设这个变量为整个工作的输入list
info_list = list()

# 此函数为跑在进程池里每个进程上的函数
def process(param1, param2):
    '''
    1. 本函数要资源完全独立，即不可使用外部类对象(例如是类成员函数)或全局变量；
    2. 本函数要包含自己的异常处理，否则每个函数抛异常都会引起进程池崩溃；
    3. 本函数的输入参数数量与调用时的tuple元素个数一致；
    4. 本函数返回值类型与result.get()出的结果一致。
    '''

    try:
        #do something
        return 123
    except:
        print(traceback.format_exc())
    
    return None


# 进程池创建时可以选择processes=[进程池数量]，如果不传，默认与电脑CPU核心数量一致
# 若执行的过程基本为计算密集型，则数量与CPU核心数量一致最为合适；
# 若包含大量IO操作，则可以适当增加processes数量
pool = Pool()

# pool.apply_async函数的返回值类型为AsyncResult类型
# 此类型调用get()函数即可等待并获取到执行的process的结果
res_list = list()
for ele in info_list:
    # 注意apply_async的第二个参数一定为tuple类型
    res_list.append(pool.apply_async(process, (ele[0], ele[1],)))

# 必须调用close表示进程池已关闭添加行为，否则可能有问题
pool.close()

# join表示本进程等待进程池全部执行完毕，与AsyncResult.get()可能功能重复
pool.join()

for res in res_list:
    # AsyncResult类型的get函数获取到process真实返回的结果。可以设置超时时间
    res.get()

# 注意：python默认函数传参都是传值；而list/tuple/dict类型则是默认传引用。
