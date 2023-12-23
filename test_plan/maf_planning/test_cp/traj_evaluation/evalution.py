#coding = utf-8

import ctypes
from ctypes import *
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('-p', "--path", help = "the first path's yaml file", type = str)
args = parser.parse_args()

ll = ctypes.cdll.LoadLibrary
lib = ll('../libtraj_evaluation.so')

fun = lib.evaTraj
fun.restype = c_double
print (fun(args.path))


