# -*- coding: utf-8 -*-

import bokeh.plotting as bkp
from bokeh.models import ColumnDataSource,TextEditor
from bokeh.io import output_notebook, push_notebook, output_file
from bokeh.layouts import layout
import pandas as pd 
from evaluation.config import conf

from bokeh.models.widgets import (DataTable, TableColumn, 
                                  StringFormatter, NumberFormatter, HTMLTemplateFormatter,
                                  StringEditor, IntEditor, NumberEditor, SelectEditor)

import sys
import os

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '../'))
from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer, FrameLayer
from plotter.event_tools import MeasureTools

score_dict={
    "bad":"60",
    "good":"100"
}
def get_template_formatter(mode):
    '''
    para了
        mode: eumn  score url
    '''
    global score_dict
    template=None
    if mode == "score":
        template = """
            <div style="background:<%= 
                (function colorfromint(){
                    if(value < bad)
                        {return('#FBA8A8')}
                    else if(value < good)
                        {return('yellow')}
                    else 
                        {return('#68FF33')}
                    }()) %>; 
                color: black"> 
            <%= value %>
            </div>
        """.replace("bad",score_dict["bad"]).replace("good",score_dict["good"])
    elif mode== "url":
        template='<a href="" target="_blank" ><%= value %></a>'
        template='''<a href=<%=
        (function f(){
            return value
        }())
        %> target="_blank" ><%= value %></a>'''
    else:
        raise("mode error")
    return HTMLTemplateFormatter(template=template)


def displayResults(odo_list, expert_list, plan_list, car_params_list, \
    plot_titles, html_path, summary_Table = None, stat_table = None):
    """
    parm:
        result_table.type:pd.DataFrame
    """
    
    if not os.path.exists(html_path) :
        os.makedirs(html_path)
    summary_Table["url"]=list()
    for test_case_name in summary_Table["test case"]:
        case_name=test_case_name.split(sep=":")
        case_name="event_{}.html".format(case_name[0])
        case_html_filename = './' + case_name
        summary_Table["url"].append(case_html_filename)
    figs=[]
    
    summary_html(html_path, summary_Table, stat_table, figs)

    if not conf["common"]["separate_html"]:
        for (odo,expert,plan,car_params,plot_title) in zip(odo_list,expert_list,plan_list,car_params_list,plot_titles):
            one_case_figs=create_case_figs(plot_title,odo,expert,plan,car_params,)
            figs.extend(one_case_figs)

        bkp.show(layout(figs), notebook_handle=True)
    else:
        
        for (odo,expert,plan,car_params,plot_title) in zip(odo_list,expert_list,plan_list,car_params_list,plot_titles):
            one_case_html(html_path,plot_title,odo,expert,plan,car_params)
       
   

def summary_html(html_path, result_table, stat_table, figs):
    '''
    param:
        result_table: 
            type:Dataframe
            a summary about infomation for all events 
    '''
    summary_html_filename=os.path.join(html_path,"summary.html")
    output_file(summary_html_filename)

    if stat_table != None:
        source = ColumnDataSource(stat_table)
        columns = [
            TableColumn(field='TYPE', title='TYPE',width=200),
            TableColumn(field='count', title='count',width=100),
            TableColumn(field='average score', title='average score',width=300),
            TableColumn(field='success rate', title='success rate',width=300),
            TableColumn(field='green:yellow:red', title='green:yellow:red',width=300)
        ]

        stat_Table = DataTable(source=source, columns=columns ,width=1000,height=200)
        figs.append(stat_Table)
    
    # print("''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''")
    # print(result_table)test for summary.html
    if len(result_table)!=0:
        source = ColumnDataSource(result_table)

        columns = [
            TableColumn(field='test case', title='test case',width=2000),
            TableColumn(field='summary score', title='score',width=500,formatter=get_template_formatter("score")),
            TableColumn(field='success', title='success', width=300),
            TableColumn(field='shift gear times', title='shift gear times',width=800),
            TableColumn(field='parking location offset', title='parking location offset',width=3000),
            TableColumn(field='obstacle min distance', title='obstacle min distance',width=1200),   
            TableColumn(field= 'url', title='url',width=1000,formatter=get_template_formatter("url"))
        ] 
        if not conf["common"]["separate_html"]:
            columns.pop()
            summary_Table = DataTable(source=source, columns=columns ,width=1500,height=1000)
            figs.append(summary_Table)  
        else :
            summary_Table = DataTable(source=source, columns=columns ,width=1500,height=1000)
            figs.append(summary_Table)
            bkp.show(layout(figs), notebook_handle=True)

def one_case_html(html_path,plot_title,odo,expert,plan,car_params,):
    case_name=plot_title[0].split(sep=":")
    case_name="event_{}.html".format(case_name[0])
    case_html_filename=os.path.join(html_path,case_name)
    output_file(case_html_filename.replace(" ","_"))
    one_html_figs=[]
    column_num=2
    for index in range(len(odo)):
        if index % column_num ==0:
            one_html_figs.append([])

        # set layers
        fig = bkp.figure(title=plot_title[index],
                        x_axis_label='x',
                        y_axis_label='y',
                        match_aspect=True,
                        width=700,
                        height=600)

        MeasureTools(fig)

        car_size_params = None
        if isinstance(car_params,list):
            car_size_params = car_params[index]
        else:
            car_size_params = car_params
        
        frame_layer = FrameLayer(fig)
        frame_layer.update(odo[index], expert[index], plan[index], car_size_params)
        fig.legend.click_policy = 'hide'
        
        one_html_figs[-1].append(frame_layer.getLayout())
    
    bkp.show(layout(one_html_figs), notebook_handle=True)

def create_case_figs(plot_title,odo,expert,plan,car_params,):
    one_case_figs=[]
    column_num=2
    for index in range(len(odo)):
        if index % column_num ==0:
            one_case_figs.append([])

        # set layers
        fig = bkp.figure(title=plot_title[index],
                        x_axis_label='x',
                        y_axis_label='y',
                        match_aspect=True,
                        width=700,
                        height=600)
        

        car_size_params = None
        if isinstance(car_params,list):
            car_size_params = car_params[index]
        else:
            car_size_params = car_params
        
        frame_layer = FrameLayer(fig)
        frame_layer.update(odo[index], expert[index], plan[index], car_size_params)
        fig.legend.click_policy = 'hide'
        
        one_case_figs[-1].append(frame_layer.getLayout())

    return one_case_figs