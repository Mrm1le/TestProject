import re

# 定义正则表达式模式来匹配数字
pattern = re.compile(r"\d+\.?\d*")

# 打开文件
with open("/home/thj/Desktop/TestProject/test_load/new_nc.log", "r") as file:
    # 初始化求和变量
    total = 0
    # 逐行读取文件
    for line in file:
        # 从行中找到所有数字
        nums = pattern.findall(line)
        # 如果第五个数字存在，将其转换为浮点数并加到总和中
        if len(nums) >= 5:
            print(float(nums[4]))
            total += float(nums[4])
    # 打印求和结果
    print("The sum of the fifth numbers in the file is:", total)
