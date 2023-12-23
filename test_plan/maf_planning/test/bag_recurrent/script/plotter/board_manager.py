
import copy
import json
import ipywidgets
import logging
import time
from functools import partial
from easydict import EasyDict
from IPython.core.display import display, HTML
from ipywidgets.widgets.widget_box import HBox, VBox

from plotter.planner_params import TUNED_PARAMS, getParamWidget
from plotter.utils.common import debounce
from plotter.utils.customize_widgets import OnOffButton

WAIT_TIME=0.2

class BoardManager():
    def __init__(self, planner_in, planner_out, edges_array, astar_planner_params, car_size_params, param_string_array) -> None:
        self.logger = logging.getLogger(__name__)
        assert len(planner_in) == len(planner_out)
        assert len(edges_array) == len(planner_out)
        assert len(astar_planner_params) == len(planner_out)
        assert len(param_string_array) == len(planner_out)
        assert len(planner_in) > 0

        self.planner_in_array = planner_in
        self.planner_out_array = planner_out
        self.edges_array = edges_array
        self.tuned_params_array = astar_planner_params
        self.car_size_params = car_size_params
        self.car_params_compare = car_size_params
        self.param_string_array = param_string_array

        self.scene_num = len(self.planner_in_array)
        self.current_scene_index = 0
        self.current_planner_in = None
        self.planner_in_copy = None
        self.planner_out_copy = None
        self.edges=[]
        self.edges_tuned=[]
        self.tuned_params = copy.deepcopy(astar_planner_params[0])
        self.param_string = self.param_string_array[0]

        self.plan_button = ipywidgets.Button(description="replan")

        self.param_slider={}
        self.allow_param_change_plan = True
        self.setParamsCallBack()

        self.updatePlannerData()

        self.edges_show_ratio = 1.0
        self.edges_tuned_show_ratio = 1.0

        self.time_slider = ipywidgets.IntSlider(
            layout=ipywidgets.Layout(width='60%'),
            min=0,
            max=self.scene_num - 1,
            value=self.current_scene_index
        )

        self.search_tree_slider = ipywidgets.FloatSlider(
            layout=ipywidgets.Layout(width='60%'),
            min = 0.0,
            max = 1.0,
            step = 0.01,
            value = self.edges_show_ratio
        )

        self.search_tree_tuned_slider = ipywidgets.FloatSlider(
            layout=ipywidgets.Layout(width='60%'),
            min = 0.0,
            max = 1.0,
            step = 0.01,
            value = self.edges_show_ratio
        )

        self.onoff_btn = OnOffButton(description='Off', icon='toggle-off')
        self.enable_compare = False



        self.plan_callback = None
        self.plot_callback = None 
        self.tree_callback = None
        self.tree_callback_tuned = None
        self.tuned_path_callback = None

        # state
        self.iteration_time_label = ipywidgets.Label(
            "iteration(base/devcar)::"+str(len(self.edges)),
            layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
        )

        self.iteration_time_label_tuned = ipywidgets.Label(
            "iteration(tuned/epcar):"+str(len(self.edges_tuned)),
            layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
        )
    
    def updatePlannerData(self):
        self.planner_in_copy = copy.deepcopy(self.planner_in_array[self.current_scene_index])
        self.planner_out_copy = self.planner_out_array[self.current_scene_index]
        self.tuned_params = copy.deepcopy(self.tuned_params_array[self.current_scene_index])
        self.param_string = self.param_string_array[self.current_scene_index]
        self.edges = self.edges_array[self.current_scene_index]
        self.edges_tuned = []
        self.allow_param_change_plan = False
        for k in self.tuned_params:
            self.param_slider[k].slider.value = self.tuned_params[k]
        
        time.sleep(WAIT_TIME+0.2)
        self.allow_param_change_plan = True


    def updateTreeState(self, is_tuned):
        self.iteration_time_label.value = "iteration(base/devcar): "+str(len(self.edges))
        self.iteration_time_label_tuned.value = "iteration(tuned/epcar): "+str(len(self.edges_tuned))
    
        slider = self.search_tree_tuned_slider if is_tuned else self.search_tree_slider
        if self.enable_compare:
            slider.value = 0.0
            return

        call_func = self.tree_callback_tuned if is_tuned else self.tree_callback
        edges = self.edges_tuned if is_tuned else self.edges
        if slider.value == 1.0:
            call_func(edges)
        else:
            slider.value = 1.0
        
        # when tuning params, the base tree is invisible in default
        if is_tuned:
            self.search_tree_slider.value = 0.0
        else:
            self.search_tree_tuned_slider.value = 0.0
    
    def setParamsCallBack(self):
        @debounce(WAIT_TIME)
        def paramFun(change, _key):
            self.tuned_params[_key] = change.new
            if self.allow_param_change_plan:
                self.plan_button.click()

        for k in self.tuned_params:
            if k not in TUNED_PARAMS:
                print("{} is not set".format(k))
            
            pb = TUNED_PARAMS[k]
            self.param_slider[k] = getParamWidget(pb)
            self.param_slider[k].setCallBack(partial(paramFun, _key= k))

    
    def setCallbacks(
        self, plan_callback, plot_callback, 
        tree_callback = None, tree_callback_tuned=None, 
        tuned_path_callback=None,path_callback = None
    ):
        self.plan_callback = plan_callback
        self.plot_callback = plot_callback 
        self.tree_callback = tree_callback 
        self.tree_callback_tuned = tree_callback_tuned
        self.tuned_path_callback = tuned_path_callback
        self.path_callback = path_callback

        self.bindPlanButtonCallback()
        self.bindTimeSliderCallback()
        self.bindSearchTreeSliderCallback()
        self.bindSearchTreeTunedSliderCallback()
        self.bindOnoffButton()
    
    def showWidgets(self):
        display(HBox((
            ipywidgets.Label(
                "frame",
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.time_slider
        )))

        display(HBox((
            ipywidgets.Label(
                "search tree",
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.search_tree_slider
        )))

        display(HBox((
            ipywidgets.Label(
                "search tree tuned",
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.search_tree_tuned_slider
        )))

        display(HBox((
            ipywidgets.Label(
                "compare devcar and epcar switch",
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.onoff_btn
        )))

        display(HBox((
            ipywidgets.Label(
                "Tuning Params",
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            # self.plan_button, 
            self.iteration_time_label, self.iteration_time_label_tuned
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


        #  for the first time to trigger
        self.time_slider.value = self.scene_num -1
        self.time_slider.value = 0
        if self.scene_num == 1:  
            #  force replot at first time
            self.plot_callback((self.planner_in_copy, self.planner_out_copy, self.car_size_params))

        self.search_tree_slider.value = 0.0
        self.search_tree_slider.value = 1.0
        
    
    def bindPlanButtonCallback(self):
        def btnFun(btn):
            if not self.enable_compare:
                self.planner_out_copy, self.car_size_params, self.edges_tuned = self.plan_callback(
                    self.combinePlannerInput(self.planner_in_copy, self.tuned_params, self.param_string)
                )
                self.updateTreeState(True)
                self.tuned_path_callback((self.planner_out_copy, self.car_size_params))

            if self.enable_compare:
                self.planner_out_copy, self.car_params_compare, self.edges = self.plan_callback(
                    self.combinePlannerInput(self.planner_in_copy, self.tuned_params, self.param_string, 'devcar')
                )
                self.updateTreeState(False)
                self.path_callback((self.planner_out_copy, self.car_params_compare))

                self.planner_out_copy, self.car_params_compare, self.edges_tuned = self.plan_callback(
                    self.combinePlannerInput(self.planner_in_copy, self.tuned_params, self.param_string, 'epcar')
                )
                self.updateTreeState(True)
                self.tuned_path_callback((self.planner_out_copy, self.car_params_compare))

        self.plan_button.on_click(btnFun)
    
    def bindTimeSliderCallback(self):
        @debounce(WAIT_TIME)
        def tslFun(t):
            index = t.new
            data = {}
            if index < 0:
                index = 0
            elif index > self.scene_num -1:
                index = self.scene_num -1
            
            self.current_scene_index = index
            self.updatePlannerData()
            if not self.enable_compare:
                self.updateTreeState(False)  
                        
                self.plot_callback((self.planner_in_copy, self.planner_out_copy, self.car_size_params))
            else:
                self.plot_callback((self.planner_in_copy, None, self.car_size_params))
                self.planner_out_copy, self.car_params_compare, self.edges = self.plan_callback(
                    self.combinePlannerInput(self.planner_in_copy, self.tuned_params, self.param_string, 'devcar')
                )
                self.updateTreeState(False)
                self.path_callback((self.planner_out_copy, self.car_params_compare))

                self.planner_out_copy, self.car_params_compare, self.edges_tuned = self.plan_callback(
                    self.combinePlannerInput(self.planner_in_copy, self.tuned_params, self.param_string, 'epcar')
                )
                self.updateTreeState(True)
                self.tuned_path_callback((self.planner_out_copy, self.car_params_compare))


        self.time_slider.observe(tslFun, names=['value'])
    
    def bindSearchTreeSliderCallback(self):
        @debounce(WAIT_TIME)
        def stsFun(change):
            max_index = int(len(self.edges) * change.new)
            max_index = max(max_index, 0)
            max_index = min(max_index, len(self.edges))
            self.tree_callback(self.edges[:max_index])
        
        self.search_tree_slider.observe(stsFun, names = ['value'])
    
    def bindSearchTreeTunedSliderCallback(self):
        @debounce(WAIT_TIME)
        def stsFun(change):
            max_index = int(len(self.edges_tuned) * change.new)
            max_index = max(max_index, 0)
            max_index = min(max_index, len(self.edges_tuned))
            self.tree_callback_tuned(self.edges_tuned[:max_index])
        
        self.search_tree_tuned_slider.observe(stsFun, names = ['value'])
    
    def bindOnoffButton(self, f=None):
        def callFunc(btn):
            if btn.description == 'On':
                self.enable_compare = True

                self.plan_button.click()

            elif btn.description == 'Off':
                self.enable_compare = False
                self.updatePlannerData()
                self.updateTreeState(False)       
                self.plot_callback((self.planner_in_copy, self.planner_out_copy, self.car_size_params))
                self.tuned_path_callback((None, None))
            else:
                raise ValueError(
                "{} not in supported OnOff button field: ('On', 'Off')".format(
                    btn.description))
            
            if f is not None:
                f(btn)
        
        self.onoff_btn.on_click(callFunc)

    @staticmethod
    def combinePlannerInput(odo, tuned_params, param_string, car_type = 'default'):
        planner_in = {
            'odo': dict(odo),
            'params': dict(tuned_params),
            'car_type': car_type,
            'param_string': param_string
        }
        in_str = json.dumps(planner_in)
        return in_str
