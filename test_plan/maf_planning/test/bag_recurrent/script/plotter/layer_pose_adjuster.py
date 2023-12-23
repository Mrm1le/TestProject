
from bokeh.io import output, push_notebook
from plotter.entity_layers import EgoCar
from plotter.composite_layers import SearchTreeLayer
import json
from easydict import EasyDict
from plotter.utils.common import convertBox, getCarCorner ,combineDict 
from plotter.utils.common import debounce, getReverseTime
import ipywidgets
from IPython.core.display import display, HTML
from ipywidgets.widgets.widget_box import HBox, VBox



from plotter.composite_layers import PlannerInputLayer, PlannerOutputLayer
from plotter.planner_params import SliderParams

import math

WAIT_TIME=0.2

class PoseAdjusterLayer:
    def __init__(self, fig,odo_string, pose):
        self.base_odo = EasyDict(json.loads(odo_string))

        self.planner_in_layer = PlannerInputLayer(fig,[{},{},{},{},{},{},{'legend_label': 'target_car'}, {'legend_label': 'init_car'}])
        self.planner_in_layer.entity_layers[7].geo.plot.visible = False
        self.planner_out_layer = PlannerOutputLayer(fig)
        self.search_tree_layer = SearchTreeLayer(fig)
        self.path_cars = []
        self.dynamic_car = EgoCar(fig, {'legend_label': 'dynamic_car'})
        self.pose = pose
        self.plan_callback = None
        self.search_edges = []
        self.sbp_res = None
    
    def trigger(self):
        theta = self.pose.theta
        self.slider_theta.slider.value = theta + 0.1
        self.slider_theta.slider.value = theta
    
    def showWidgets(self):
        display(HBox((
            ipywidgets.Label(
                self.slider_ego.label,
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.slider_ego.slider
        )))
        display(HBox((
            ipywidgets.Label(
                self.slider_x.label,
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.slider_x.slider
        )))
        display(HBox((
            ipywidgets.Label(
                self.slider_y.label,
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.slider_y.slider
        )))
        display(HBox((
            ipywidgets.Label(
                self.slider_theta.label,
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.slider_theta.slider
        )))
        display(HBox((
            ipywidgets.Label(
                self.slider_tree.label,
                layout=ipywidgets.Layout(display="flex", justify_content="flex-end")
            ),
            self.slider_tree.slider
        )))
        pass

    def setPlanCallback(self,plan_callback):
        self.plan_callback = plan_callback

    def setSlider(self,min_x, min_y, max_x, max_y):
        self.slider_x = SliderParams("x", self.pose.x, min_x, max_x, 0.1)
        self.slider_y = SliderParams("y", self.pose.y, min_y, max_y, 0.1)
        self.slider_theta = SliderParams("theta",self.pose.theta, -math.pi, math.pi, 0.01)
        self.slider_ego = SliderParams("dynamic path", 0, 0, 0, 1)
        self.slider_tree = SliderParams("search tree", 0, 0, 0, 1)

        @debounce(WAIT_TIME)
        def sliderXFunc(change):
            self.pose.x = change.new
            self.sliderDefaultCallback()
        
        @debounce(WAIT_TIME)
        def sliderYFunc(change):
            self.pose.y = change.new
            self.sliderDefaultCallback()
        
        @debounce(WAIT_TIME)
        def sliderTFunc(change):
            self.pose.theta = change.new
            self.sliderDefaultCallback()
        
        self.slider_x.setCallBack(sliderXFunc)
        self.slider_y.setCallBack(sliderYFunc)
        self.slider_theta.setCallBack(sliderTFunc)

        def sliderEgoFunc(change):
            if len(self.path_cars) ==0 or change.new > len(self.path_cars)-1:
                return
            car_xy = self.path_cars[change.new]
            self.dynamic_car.updateUtil((car_xy, self.car_params))
            push_notebook()

        self.slider_ego.setCallBack(sliderEgoFunc)
        
        @debounce(WAIT_TIME)
        def sliderTreeFunc(change):
            if change.new > len(self.search_edges)-1:
                return
            
            self.search_tree_layer.updateUtil(self.search_edges[:change.new])
            push_notebook()
        
        self.slider_tree.setCallBack(sliderTreeFunc)
    
    def sliderDefaultCallback(self):
        res, search_edges = self.plan_callback(self.pose)
        self.updateUtil(self.pose, res[0], res[1], search_edges)
        push_notebook()
        pass

    def updateUtil(self, pose, sbp_str, car_params, search_edges):
        self.car_params = EasyDict(json.loads(car_params))
        #  update pose
        self.base_odo.target_state.path_point.x  = pose.x
        self.base_odo.target_state.path_point.y  = pose.y
        self.base_odo.target_state.path_point.theta  = pose.theta
        self.planner_in_layer.updateUtil((self.base_odo, self.car_params))
        self.dynamic_car.updateUtil((self.base_odo.target_state.path_point, self.car_params))

        # update res
        sbp_res = EasyDict(json.loads(sbp_str))
        
        # delete none
        rx = []
        ry = []
        rphi = []
        for i in range(len(sbp_res.x)):
            if sbp_res.x[i] is None:
                continue
            
            if sbp_res.y[i] is None:
                continue
            
            if sbp_res.phi[i] is None:
                continue
            
            rx.append(sbp_res.x[i])
            ry.append(sbp_res.y[i])
            rphi.append(sbp_res.phi[i])
        
        if len(rx) > 0:
            print("reverse time:", getReverseTime(rx, ry))
        
        sbp_res.x = rx
        sbp_res.y = ry
        sbp_res.phi = rphi
        # end
        self.sbp_res = sbp_res


        
        self.planner_out_layer.updateUtil((sbp_res, self.car_params))

        if len(sbp_res.x) ==0:
            self.path_cars = []
            self.slider_ego.slider.max = 0
            self.slider_ego.slider.value = 0
        else:
            self.path_cars= [
            {
                'x':sbp_res.x[i],
                'y':sbp_res.y[i],
                'theta':sbp_res.phi[i]

                } for i in range(len(sbp_res.x)-1,-1,-1)
            ]
            self.slider_ego.slider.max = len(self.path_cars) - 1
            self.slider_ego.slider.value = 0
        
        # update searchtree
        self.search_edges = search_edges
        self.slider_tree.slider.max = len(search_edges) - 1 if len(search_edges)>0 else 0
        self.slider_tree.slider.value = 0

        push_notebook()



    


