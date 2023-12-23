import sys
import os

args = ["","opt","real","opt_real"]
for param in args:
    os.system("python3 cp_evaluation.py "+param)