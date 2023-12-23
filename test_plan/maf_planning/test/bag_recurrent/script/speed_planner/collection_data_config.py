import sys, os
import json
from easydict import EasyDict
sys.path.insert(0, os.path.join(os.getcwd(), "../../lib"))
sys.path.insert(0, os.path.join(os.getcwd(), "../"))
import csv
import numpy as np
import py_parking_plotter as apa_plan

from plotter.event_tools import MeasureTools
from plotter.entity_layers import ObsPoints,PlannerPathLine,PlannerBox
from plotter.utils.common import GEO_TYPE, getCurvature

from plotter.load_bag import DataLoader,DataGeneratorBase
import rosbag


# display type: "4"  for rectangle, "6" for hexagon， "14" for tetradecagon
class PathGenerator(DataGeneratorBase):
    def __init__(self, data,ind=0):
        self.margin_paras=[]
        tra,ts=self._convert(data)
        self.ind=ind
        self.is_tra_margin_debug=0
        super().__init__(tra,ts)
    
    def _convert(self, data):
        if data is None:
            return [[],[],[],[]]
        ts_margin=[]
        tra_margin=[]
        tra_differ=[]
        ts_differ=[]
        self.margin_index_list=[]
        margin_paras=[]
        for d in data:
            first=10
            if tra_differ==[]:first=0
            json_str = d[0].extra.json if hasattr(d[0].extra, 'json') else ''
            extra = json.loads(json_str) if json_str else {}
            if type(extra) != dict or "speed_margin_debug" not in extra:
                continue
            if len(d[0].trajectory.path) == 0:
                continue
            xs=[]
            ys=[]
            theta=[]
            for p in d[0].trajectory.path:
                if p.position_enu.x :
                    xs.append(p.position_enu.x)#xs[x1,x2,...,xn]
                    ys.append(p.position_enu.y)
                    theta.append(p.heading_yaw)
            if not xs: continue
            if  "plan_param" not in extra or not extra['plan_param']: 
                margin_index=-1
            else:
                plan_param=extra['plan_param']
                margin_index=plan_param['gear_change_index']
            xs=xs[first:margin_index]
            ys=ys[first:margin_index]
            theta=theta[:margin_index]
            if tra_differ==[] or tra_differ[-1][0][0]!=xs[0]:
                ts_differ.append(d[1])
                tra_differ.append((xs,ys,theta))
                self.margin_index_list.append(margin_index)
            if extra['speed_margin_debug']:
                margin_para=extra['speed_margin_debug']
                margin_paras.append(margin_para)
                ts_margin.append(d[1])
                tra_margin.append((xs,ys,theta))
        if tra_margin:
            print("-----Get speed_margin_debug trajectories number:",len(tra_margin))
            tra=tra_margin
            ts=ts_margin
            self.is_tra_margin_debug=1
            self.margin_paras=margin_paras
        else:
            print("-----Get dfferent_planning trajectories number:",len(tra_differ))
            tra=tra_differ
            ts=ts_differ
            self.margin_paras=[""] * len(ts)
        return (tra,ts)#(xys, ts)

    def get_is_tra_margin_debug(self):
        return self.is_tra_margin_debug
    
    def get_index_list(self):
        return self.margin_index_list

    def get_select_ks(self,ind):
        if ind:
            self.ind=ind
        ks = getCurvature(self.xys[self.ind][0], self.xys[self.ind][1])
        delta_len = len(self.xys[self.ind][0]) - len(ks)
        head_ks = [0 for i in range(delta_len)]
        head_ks.extend(ks)
        ks = head_ks
        return ks

    def get_margin_paras(self):
        return self.margin_paras
    def get_ts(self):
        return self.ts
    def get_this_ts(self,ind_ts):
        return self.ts[ind_ts]
    def get_traj(self):
        return self.xys


class EgoinfoGenerator(DataGeneratorBase):
    def __init__(self, data):
        v,self.pos,ts=self._convert(data)
        super().__init__(v,ts)
  
    def _convert(self, data):
        if data is None:
            return [[],[]]
        pos=[]
        v=[]
        ts=[]    
        for d in data:
            vx=d[0].velocity.velocity_local.vx
            vy=d[0].velocity.velocity_local.vy
            v_tmp=(vx**2+vy**2)**0.5
            px=d[0].position.position_local.x
            py=d[0].position.position_local.y
            if v and (px or py):
                v.append(v_tmp)
                pos.append[(px,py)]
                ts.append(d[1])
        return (v,pos,ts)
    
    def get_ego_pos(self):
        return self.pos


class EnvGenerator(DataGeneratorBase):
    def __init__(self, data):
        Env,ts=self._convert(data)
        super().__init__(Env,ts)
  
    def _convert(self, data):
        if data is None:
            return [[],[],0,[]]
        ts=[]
        Env=[]
        for d in data:
            ts.append(d[1])
            xs=[]
            ys=[]
            zs=[]
            ls=[]
            for line in d[0].ground_line.ground_line_data:
                for p in line.local_points_fusion:
                    xs.append(p.x)
                    ys.append(p.y)
                    zs.append(p.z)
                ls.append((xs,ys,zs))
            Env.append(ls)
        if len(Env) == 0:
            Env = [([()])]
        return (Env,ts)#(xys, ts)

