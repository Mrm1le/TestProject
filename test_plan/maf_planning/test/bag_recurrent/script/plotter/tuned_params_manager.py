
import copy
import json
import ipywidgets
import logging
import time
from functools import partial
from easydict import EasyDict
from IPython.core.display import display, HTML
from ipywidgets.widgets.widget_box import HBox, VBox
from bokeh.plotting import ColumnDataSource
from bokeh.models import HoverTool, Slider, CustomJS, Div
from plotter.utils.common import convertBox, getCarCorner ,combineDict 
from bokeh.io import output_notebook, push_notebook

from plotter.planner_params import TUNED_PARAMS, getParamWidget
from plotter.utils.common import debounce
from plotter.utils.customize_widgets import OnOffButton

WAIT_TIME=0.2

class BoardManager():
    def __init__(self,fig, planner_in, planner_out, astar_planner_params, car_size_params) -> None:
        self.logger = logging.getLogger(__name__)
        self.fig = fig

        self.planner_in = planner_in
        self.planner_out = planner_out
        self.tuned_params = astar_planner_params
        self.astar_planner_params = astar_planner_params
        self.car_size_params = car_size_params
        self.allow_param_change_plan = False

        
        self.tuned_params_copy = copy.deepcopy(astar_planner_params)
        

        self.param_slider={}
        self.car_slider = ipywidgets.IntSlider(
            layout=ipywidgets.Layout(width='60%'),
            min=0,
            max=0,
            value=0
        )

        self.plan_callback = None
        self.plot_callback = None 
        self.dynamicSliderCall = None
        
        self.reset_plan = ipywidgets.Button(description="reset params")
        
        self.polygon_xs_array, self.polygon_ys_array = 'pts_xs_array', 'pts_ys_array'
        self.polygon_datas={}
    
    def plotScene(self):
        if self.plot_callback is not None:
            self.plot_callback((self.planner_in, self.planner_out))
    
    def setCallbacks(
        self, plan_callback, plot_callback, dynamicSliderCall
    ):
        self.plan_callback = plan_callback
        self.plot_callback = plot_callback 
        self.dynamicSliderCall = dynamicSliderCall
        
        self.setParamsCallBack()
        self.setResetCallback()
        self.bindCarSliderCallback()
    
    def setResetCallback(self):
        def callFunc(btn):
            self.tuned_params_copy = copy.deepcopy(self.tuned_params)
            for k in self.tuned_params_copy:  
                self.param_slider[k].slider.value = self.tuned_params_copy[k]
            self.replan()
            self.plotScene()
            
        self.reset_plan.on_click(callFunc)
            
        
    def replan(self):
        planner_in = self.combinePlannerInput(self.planner_in, self.tuned_params_copy, "")
        self.planner_out = self.plan_callback(planner_in)
        self.resetSliderValues(self.planner_out)
        
        
    def setParamsCallBack(self):
        @debounce(WAIT_TIME)
        def paramFun(change, _key):
            self.tuned_params_copy[_key] = change.new
            if self.allow_param_change_plan:
                self.replan()
                self.plotScene()

        for k in self.tuned_params_copy:
            if k not in TUNED_PARAMS:
                print("{} is not set".format(k))
            
            pb = TUNED_PARAMS[k]
            self.param_slider[k] = getParamWidget(pb)
            self.param_slider[k].setCallBack(partial(paramFun, _key= k))
            self.param_slider[k].slider.value = self.tuned_params[k]
        
        time.sleep(WAIT_TIME+0.2)       
        self.allow_param_change_plan = True

    
    def showWidgets(self):
        display(HBox((
            ipywidgets.Label(
                "frame",
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.car_slider
        )))
        display(HBox((
            ipywidgets.Label(
                "reset all params",
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.reset_plan
        )))

        display(
            VBox(
                tuple(
                    HBox((
                        ipywidgets.Label(
                            param.label,
                            layout=ipywidgets.Layout(display="flex", justify_content="flex-end", min_width="20ex !important")
                        ),
                        param.slider
                    )) for param in self.param_slider.values() 
                )
            )
        )
        
        self.plotScene()
        self.resetSliderValues(self.planner_out)
        
    def bindCarSliderCallback(self):
        # @debounce(0.1)
        def tslFun(t):
            index = t.new
            self.dynamicSliderCall((
                self.polygon_datas['pts_xs_array'][index],
                self.polygon_datas['pts_ys_array'][index]
            ))
        self.car_slider.observe(tslFun, names=['value'])
        
    def resetSliderValues(self, sbp_res):
        if sbp_res is None or len(sbp_res.x)  ==0:
            self.car_slider.max = 0
            self.car_slider.value = 0
            return
        pts_xs = sbp_res.x
        pts_ys = sbp_res.y
        pts_thetas = sbp_res.phi
        
        car_xs=[]
        car_ys=[]
        for i in range(len(pts_xs)-1, -1, -1):
            carxx, caryy = getCarCorner(pts_xs[i], pts_ys[i], pts_thetas[i], self.car_size_params)     
            car_xs.append([[[carxx]]])
            car_ys.append([[[caryy]]])
        
        self.polygon_datas={
            self.polygon_xs_array: car_xs,
            self.polygon_ys_array: car_ys,
        }
        self.car_slider.max = len(car_xs)-1
        self.car_slider.value = 0
    
        
    def combinePlannerInput(self,odo, tuned_params, param_string, car_type = 'default'):
        planner_in = {
            'odo': dict(odo),
            'params': dict(tuned_params),
            'car_type': car_type,
            'param_string': param_string
        }
        in_str = json.dumps(planner_in)
        return in_str

    
