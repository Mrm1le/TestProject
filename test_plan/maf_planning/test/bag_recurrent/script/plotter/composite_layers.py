from abc import ABC, abstractmethod

from bokeh.io import output
from plotter.entity_layers import ObsLines, MapLines, ObsPoints, ObsBoxes, SpeedMarginBoxes
from plotter.entity_layers import PlannerBoundary, EgoCar, TLines, SVLine
from plotter.entity_layers import PlannerBox, PlannerPathLine, EnvironmentSlotLine
from bokeh.plotting import ColumnDataSource
from bokeh.layouts import column, row
from bokeh.models import HoverTool, Slider, CustomJS, Div
from plotter.utils.common import combineDict, getReverseTime, filterNone
from plotter.basic_layers import PointsLayer, CurveLayer

import json
from easydict import EasyDict
from plotter.utils.common import convertBox, getCarCorner ,combineDict 



class CompositeLayerBase(ABC):
    def __init__(self) -> None:
        self.entity_layers = []
    
    def update(self, args):
        if args is None:
            for i in range(len(self.entity_layers)):
                self.entity_layers[i].updateUtil(None)
            return
        assert len(args) == len(self.entity_layers)
        for index, arg in enumerate(args):
            self.entity_layers[index].updateUtil(arg)

    def updateUtil(self, data):
        data_converted = self.convert(data)
        self.update(data_converted)
    
    @abstractmethod
    def convert(self, data):
        pass

class SpeedCurveLayer(CompositeLayerBase):
    def __init__(self, fig, params=[]) -> None:
        super().__init__()
        layer_num = 1
        if params is None or len(params) == 0:
            params = [{} for i in range(layer_num)]
        self.entity_layers.extend([
            SVLine(fig, params[0])
        ])


    def convert(self, data):
        if data is None:
            return data

        data_converted=(
            data,
        )

        return data_converted

class SpeedMarginLayer(CompositeLayerBase):
    def __init__(self, fig, params=[]) -> None:
        super().__init__()
        if params is None or len(params) == 0:
            params = [{} for i in range(8)]
        self.entity_layers.extend([
            # obs
            ObsPoints(fig, params[0]), ObsLines(fig, params[1]),  ObsBoxes(fig,params[2]),
            PlannerPathLine(fig, params[3]), PlannerBox(fig, params[4])
        ])
        self.entity_layers[4].geo.plot.visible = True


    def convert(self, data):
        if data is None:
            return data
        
        speed_margin_debug, car_params = data

        path= [
            {
                'x':speed_margin_debug.xs[i],
                'y':speed_margin_debug.ys[i],
                'theta':speed_margin_debug.thetas[i]
            } for i in range(len(speed_margin_debug.xs))
        ]
                
        data_converted=(
            # obs
            speed_margin_debug.obs_pts,
            speed_margin_debug.obs_lines,
            speed_margin_debug.obs_boxes,
            path, 
            (path, car_params)
        )

        return data_converted

class SpeedCurveLayer(CompositeLayerBase):
    def __init__(self, fig, params=[]) -> None:
        super().__init__()
        layer_num = 1
        if params is None or len(params) == 0:
            params = [{} for i in range(layer_num)]
        self.entity_layers.extend([
            SVLine(fig, params[0])
        ])

    def convert(self, data):
        if data is None:
            return data
        data_converted=(
            data,
        )
        return data_converted
    
class MultiSpeedCurveLayer(CompositeLayerBase):
    def __init__(self, fig, params=[],layer_num=1) -> None:
        super().__init__()
        layer_num = layer_num
        if params is None or len(params) == 0:
            params = [{} for i in range(layer_num)]
        self.entity_layers.extend([
            SVLine(fig, params[i]) for i in range(layer_num)
        ])
    def convert(self, data):
        if data is None:
            return data
        data_converted=(
            data
        )
        return data_converted  

class SceneSpeedMarginLayer(CompositeLayerBase):
    def __init__(self, fig, params=[]) -> None:
        super().__init__()
        if params is None or len(params) == 0:
            params = [{} for i in range(4)]
        self.entity_layers.extend([
            # obs
            ObsPoints(fig, params[0]), 
            PlannerPathLine(fig, params[1]),
            PlannerBox(fig, params[2]),
            SpeedMarginBoxes(fig, params[3]),
        ])
        self.entity_layers[2].geo.plot.visible = True
        self.entity_layers[3].geo.plot.visible = False


    def convert(self, data):
        if data is None:
            return data
        
        path_debugs,env_debugs,car_params, polygons = data
        env_points=[
            {
            "id_":0,
            "x_":env_debugs[0][0][i],#,
            "y_":env_debugs[0][1][i],
            } for i in range(len(env_debugs[0][0]))
        ]

        path=[
            {
            'x':path_debugs[0][i],#xs[i]
            'y':path_debugs[1][i],#ys[i]
            'theta':path_debugs[2][i]#theta[i]
            } for i in range(len(path_debugs[0]))
        ]

        poly_xs = []
        poly_ys = []
        for p in polygons:
            xs = [pi.x for pi in p]
            ys = [pi.y for pi in p]
            poly_xs.append([[xs]])
            poly_ys.append([[ys]])
        
        data_converted=(
            env_points,
            path,
            (path, car_params),
            (poly_xs, poly_ys)
        )

        return data_converted

class PlannerInputLayer(CompositeLayerBase):
    def __init__(self, fig, params=[]) -> None:
        super().__init__()
        if params is None or len(params) == 0:
            params = [{} for i in range(8)]
        self.entity_layers.extend([
            # obs
            ObsPoints(fig, params[0]), ObsLines(fig, params[1]), MapLines(fig,params[2]), ObsBoxes(fig,params[3]),
            # 
            PlannerBoundary(fig,params[4]), TLines(fig,params[5]), 
            EgoCar(fig,params[6]), EgoCar(fig,params[7])
            # EgoCar(fig, {'legend_label': 'target_car'}), EgoCar(fig, {'legend_label': 'init_car'})
            # 
        ])
    
    def convert(self, data):
        if data is None:
            return data
        
        odo, car_params = data
        
        if odo is None or car_params is None:
            return None
        
        data_converted=(
            # obs
            odo.points,
            odo.obstacle_lines,
            odo.lines,
            odo.obstacle_boxs,
            # 
            odo.map_boundary,
            odo.T_lines,
            (odo.init_state.path_point, car_params),
            (odo.target_state.path_point, car_params)
        )

        return data_converted

class PlannerOutputLayer(CompositeLayerBase):
    def __init__(self, fig, path_params=None, planner_box_params=None) -> None:
        super().__init__()
        self.entity_layers.extend([
           PlannerPathLine(fig, path_params), PlannerBox(fig, planner_box_params)
        ])

    
    def convert(self, data):
        if data is None:
            return data
        
        plan_res, car_params = data

        if plan_res is None:
            return plan_res

        path= [
            {
                'x':plan_res.x[i],
                'y':plan_res.y[i],
                'theta':plan_res.phi[i]
                # 'steer':planner_result.steer[i],
                # 'v':planner_result.v[i],
                # 'a':planner_result.a[i]
                } for i in range(len(plan_res.x))
        ]
        
        data_converted=(
            path, 
            (path, car_params)
        )

        return data_converted

class EnvironmentSlotLayer(CompositeLayerBase):
    def __init__(self, fig, path_params=None) -> None:
        super().__init__()
        self.entity_layers.extend([
           EnvironmentSlotLine(fig, path_params)
        ])

    def convert(self, data):
        if data is None:
            return data
        
        plan_res = data

        if plan_res is None:
            return plan_res
        try:
            debug_json = json.loads(plan_res.debug_string)
        except Exception as e:
            return None

        path = [
            {
                'x':debug_json["environment_of_slot"][i]["x_"],
                'y':debug_json["environment_of_slot"][i]["y_"],
                'theta': 0.0
                # 'steer':planner_result.steer[i],
                # 'v':planner_result.v[i],
                # 'a':planner_result.a[i]
                } for i in range(len(debug_json["environment_of_slot"]))
        ]

        print("environment_of_slot path!!!!!")
        print(len(path))
        print(path)
        
        data_converted=(
            path,
        )

        return data_converted

class PerfPlotLayer(CompositeLayerBase):
    def __init__(self,fig, params = [None, None, None, None, None]) -> None:
        super().__init__()
        
        self.plan_out = PlannerOutputLayer(fig)
        
        self.entity_layers.extend([
            ObsLines(fig, params[0]), MapLines(fig,params[1]),
            PlannerBoundary(fig,params[2]), EgoCar(fig,params[3]),
        ])
        
        #  planner points
        self.fig=fig
        self.xs, self.ys, self.color = 'pts_xs', 'pts_ys', "pts_color"
        self.thetas = 'pts_thetas'
        self.target_xs, self.target_ys = 'target_xs', 'target_ys'
        self.success_low_reverse = "darkgreen"      # reverse time <= 9
        self.success_high_reverse = "darkred"     # reverse time > 9
        self.failed_color = "blue"
        self.dot_data_source = ColumnDataSource(data={
            self.xs: [],
            self.ys: [],
            self.thetas: [],
            self.color: []
        })
        
        fig_params = {
            'legend_label': 'test pose',
            'size': 5,
            'alpha': 0.5,
            'line_color': None
        } 
        new_params = combineDict(fig_params, params[4])
        self.plot_success = self.fig.scatter(self.xs,
            self.ys,
            color = self.color,
            source=self.dot_data_source,
            **new_params)
        
        target_params = {
            'legend_label': 'target_pose',
            'size': 10,
            'color': 'firebrick',
        } 
        self.target_source = ColumnDataSource(data={
            self.target_xs: [],
            self.target_ys: [],
        })
        self.fig.asterisk(
            self.target_xs, 
            self.target_ys, 
            source=self.target_source,
            **target_params
        )
        
        self.br = []
        
        # add egocar slider
        self.polygon_xs, self.polygon_ys = 'pts_xs', 'pts_ys'
        self.polygon_xs_array, self.polygon_ys_array = 'pts_xs_array', 'pts_ys_array'

        self.polygon_datasource = ColumnDataSource(data={
            self.polygon_xs: [],
            self.polygon_ys: [],
            self.polygon_xs_array: [],
            self.polygon_ys_array: []
        })
        polygon_params = {
            'legend_label': 'dynamic_car',
            'line_color': 'green', 
            'fill_color': 'green',
            'alpha': 0.5, 
            'line_width': 3
        } 
        self.dynamic_car = self.fig.multi_polygons(
            self.polygon_xs, self.polygon_ys,
            source = self.polygon_datasource,
            **polygon_params
        )
        
        
        #  add hover tool

        # self.hover_tool = HoverTool(
        #     renderers = [self.plot_success],
        #     line_policy='interp',
        #     point_policy='snap_to_data',
        #     tooltips = [
        #         ('pts_thetas', '@pts_thetas')
        #     ]                    
        # )
        # self.fig.add_tools(self.hover_tool)
        
        self.car_slider = Slider(start = 0, end = 1, value = 0, step = 1, title = "move nothing")
        self.hor_div = Div(width=400, height=20, height_policy="fixed")
        self.ver_div = Div(width=20, height=40, height_policy="fixed")
    
    def getLayout(self):
        return row(column(self.hor_div, self.car_slider, self.fig), self.ver_div)
    
    def addDynamicSlider(self, car_size_params, sbp_res):
        steps = len(sbp_res.x)
        pts_xs = sbp_res.x
        pts_ys = sbp_res.y
        pts_thetas = sbp_res.phi
        
        car_xs=[]
        car_ys=[]
        for i in range(len(pts_xs)-1, -1, -1):
            carxx, caryy = getCarCorner(pts_xs[i], pts_ys[i], pts_thetas[i], car_size_params)     
            car_xs.append([[[carxx]]])
            car_ys.append([[[caryy]]])
        
        self.polygon_datasource.data.update({
            self.polygon_xs: car_xs[0],
            self.polygon_ys: car_ys[0],
            self.polygon_xs_array: [car_xs],
            self.polygon_ys_array: [car_ys],
        })
        # print(self.polygon_datasource.data)
        
        self.car_slider = Slider(start = 0, end = steps-1, value = 0, step = 1, title = "move car")
        callback = CustomJS(args=dict(polygon_datasource = self.polygon_datasource),
                            code = """
    const step = cb_obj.value;
    const data = polygon_datasource.data;
    polygon_datasource.data['pts_xs'] = data['pts_xs_array'][0][step];
    polygon_datasource.data['pts_ys'] = data['pts_ys_array'][0][step];
    polygon_datasource.change.emit();
                            """)
        self.car_slider.js_on_change('value', callback)
                
    def setHoverTool(self, data):
        car_params = EasyDict(json.loads(data.debug_info[1]))
        plan_ress = [EasyDict(json.loads(p.debug_info)) if p.is_success else None for p in data.br ]
        
        pts = []
        
        
        l_xs = []
        l_ys = []
        b_xs = []
        b_ys = []
        
        for res in plan_ress:
            if res is None:
                l_xs.append([])
                l_ys.append([])
                b_xs.append([])
                b_ys.append([])
                continue
            pts_xs = res.x
            pts_ys = res.y
            pts_thetas = res.phi
            
            car_xs=[]
            car_ys=[]
            for i in range(len(pts_xs)):
                carxx, caryy = getCarCorner(pts_xs[i], pts_ys[i], pts_thetas[i], car_params)     
                car_xs.append([[carxx]])
                car_ys.append([[caryy]])
            
            l_xs.append(pts_xs)
            l_ys.append(pts_ys)
            b_xs.append(car_xs)
            b_ys.append(car_ys)
            # pts.append([pts_xs, pts_ys, car_xs, car_ys ])
        self.pathline_datasource = ColumnDataSource(data={
            "l_xs": l_xs,
            "l_ys": l_ys,
            "b_xs": b_xs,
            "b_ys": b_ys
        })
            
        code = """
console.log("haha");
const data = source.data;

const indices = cb_data.index.indices;
const path_data = {'pts_xs': [], 'pts_ys': []};
const bounding_data = {'pts_xs': [], 'pts_ys': []};
console.log(pts);
console.log(pts[0][0]);

if(indices.length > 1)
    const index = indices[0];
    if( pts[index].length <1 ){
        continue;
    }
    path_source.data['pts_xs'] = data['l_xs'][index];
    path_source.data['pts_ys'] = data['l_ys'][index];
    path_source.data['pts_xs'] = data['b_xs'][index];
    bounding_source.data['pts_ys'] = data['b_ys'][index];
    
    path_source.change.emit();
    bounding_source.change.emit();
}
"""
        callback = CustomJS(
            args=dict(
                path_source = self.plan_out.entity_layers[0].geo.data_source,
                bounding_source = self.plan_out.entity_layers[0].geo.data_source,
                source = self.pathline_datasource
            ),
            code=code
        )
        self.hover_tool = HoverTool(
            callback=callback,
            renderers = [self.plot_success],
            line_policy='interp',
            point_policy='snap_to_data',
            tooltips = [
                ('pts_thetas', '@pts_thetas')
            ]                    
        )
        self.fig.add_tools(self.hover_tool)
    
    def convert(self, data):
        if data is None:
            return data
        
        odo= EasyDict(json.loads(data.debug_info[0]))
        car_size_params = EasyDict(json.loads(data.debug_info[1]))
        data_converted=(
            odo.obstacle_lines,
            odo.lines,
            odo.map_boundary,
            (odo.init_state.path_point, car_size_params)
        )
        
        br = data.br
        if br is None:
            return data_converted
        
        pts_xs = []
        pts_ys = []
        pts_colors = []
        pts_thetas = []
        self.br = br
        
        # self.setHoverTool(data)
        
        sbp_res = None
        
        for b in br:
            if not b.is_success:
                continue
            sbp_res = EasyDict(json.loads(b.debug_info))
            break
        
        rx = []
        ry = []
        rphi = []
        if sbp_res is not None:
            # delete none
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
            
            sbp_res.x = rx
            sbp_res.y = ry
            sbp_res.phi = rphi
            # end
            self.plan_out.updateUtil((
                EasyDict(json.loads(b.debug_info)),
                car_size_params
            ))
            self.addDynamicSlider(car_size_params, sbp_res)
        
        for b in br:
            pts_xs.append(b.point.x)
            pts_ys.append(b.point.y)
            pts_thetas.append(b.point.theta)
            if b.is_success:
                xs, ys, phis = filterNone(EasyDict(json.loads(b.debug_info)))
                reverse_num = getReverseTime(xs, ys)
                pts_colors.append(self.success_low_reverse if reverse_num <= 9 else self.success_high_reverse )
            else:
                pts_colors.append(self.failed_color)

            
        self.dot_data_source.data.update({
            self.xs: pts_xs,
            self.ys: pts_ys,
            self.thetas: pts_thetas,
            self.color: pts_colors
        })
        
        target_xs = [odo.init_state.path_point.x]
        target_ys = [odo.init_state.path_point.y]
        self.target_source.data.update({
            self.target_xs: target_xs,
            self.target_ys: target_ys
        })

        return data_converted
        
        
        

class SearchTreeLayer():
    count = 0
    def __init__(self, fig, params = None):
        SearchTreeLayer.count += 1
        self.fig = fig
        self.xs, self.ys = 'pts_xs', 'pts_ys'
        self.edge_cost = 'edge_cost'
        self.start_traj_cost = 'start_traj_cost'
        self.start_heuristic_cost = 'start_heuristic_cost'
        self.end_traj_cost = 'end_traj_cost'
        self.end_heuristic_cost = 'end_heuristic_cost'
        self.data_source = ColumnDataSource(data={
            self.xs: [],
            self.ys: [],
            self.edge_cost: [],
            self.start_traj_cost: [],
            self.start_heuristic_cost: [],
            self.end_traj_cost: [], 
            self.end_heuristic_cost: []
        })


        fig_params = {
            'legend_label': 'search_tree_'+str(SearchTreeLayer.count),
            'line_color': 'grey',
            'line_width': 1,
            'alpha':0.2
        } 

        new_params = combineDict(fig_params, params)


        self.plot = self.fig.multi_line(
            self.xs,
            self.ys,
            source=self.data_source,
            **new_params
        )

        self.hover_tool = HoverTool(
            renderers = [self.plot],
            line_policy='interp',
            point_policy='snap_to_data',
            tooltips = [
                ('pts_xs', '@pts_xs'),
                ('pts_ys', '@pts_ys'),
                ('edge_cost', '@edge_cost'),
                ('start_traj_cost', '@start_traj_cost'),
                ('start_heuristic_cost', '@start_heuristic_cost'),
                ('end_traj_cost', '@end_traj_cost'),
                ('end_heuristic_cost', '@end_heuristic_cost')
            ]                    
        )
        self.fig.add_tools(self.hover_tool)

    def update(self, args):
        pts_xs, pts_ys, edge_cost, start_traj_cost, start_heuristic_cost, end_traj_cost, end_heuristic_cost = args
        self.data_source.data.update({
            self.xs: pts_xs,
            self.ys: pts_ys,
            self.edge_cost: edge_cost,
            self.start_traj_cost: start_traj_cost,
            self.start_heuristic_cost: start_heuristic_cost,
            self.end_traj_cost: end_traj_cost, 
            self.end_heuristic_cost: end_heuristic_cost
        })

    def updateUtil(self, data):
        data_converted = self.convert(data)
        self.update(data_converted)
    
    def convert(self, data):
        pts_xs = []
        pts_ys = []
        edge_cost = []

        start_traj_cost = []
        start_heuristic_cost = []

        end_traj_cost = []
        end_heuristic_cost = []

        if data is None:
            return (
                        pts_xs, pts_ys, edge_cost,
                        start_traj_cost, start_heuristic_cost,
                        end_traj_cost, end_heuristic_cost
                    ) 
        
        for edges in data:
            for edge in edges:
                pts_xs.append([edge.start_node.x, edge.end_node.x])
                pts_ys.append([edge.start_node.y, edge.end_node.y])
                edge_cost.append(edge.edge_cost)
                start_traj_cost.append(edge.start_node.traj_cost)
                start_heuristic_cost.append(edge.start_node.heuristic_cost)
                end_traj_cost.append(edge.end_node.traj_cost)
                end_heuristic_cost.append(edge.end_node.heuristic_cost)

        
        return (
                    pts_xs, pts_ys, edge_cost,
                    start_traj_cost, start_heuristic_cost,
                    end_traj_cost, end_heuristic_cost
                )  


class FrameLayer:
    def __init__(self, fig) -> None:
        self.fig=fig
        self.in_layer = PlannerInputLayer(fig,[{},{},{},{},{},{},{'legend_label': 'target_car'}, {'legend_label': 'init_car'}])
        self.out_layer = PlannerOutputLayer(fig,  {'line_color': 'green', 'legend_label':'planner path'}, {'line_color': 'mediumseagreen', 'legend_label':'car polygon'})
        self.out_layer_pro = PlannerOutputLayer(fig,  {'line_color': 'red', 'legend_label':'expert planner path'}, {'line_color': 'purple', 'legend_label':'expert car polygon'})
        self.environment_slot_layer = EnvironmentSlotLayer(fig,  {'line_color': 'blue', 'legend_label':'environment_slot'})
        #self.out_layer_pro.entity_layers[0].geo.plot.visible = False
        
        
        self.polygon_xs, self.polygon_ys = 'pts_xs', 'pts_ys'
        self.polygon_xs_array, self.polygon_ys_array = 'pts_xs_array', 'pts_ys_array'
        self.polygon_datasource = ColumnDataSource(data={
            self.polygon_xs: [],
            self.polygon_ys: [],
            self.polygon_xs_array: [],
            self.polygon_ys_array: []
        })
        self.polygon_datasource_pro = ColumnDataSource(data={
            self.polygon_xs: [],
            self.polygon_ys: [],
            self.polygon_xs_array: [],
            self.polygon_ys_array: []
        })
        polygon_params = {
            'legend_label': 'dynamic_car',
            'line_color': 'seagreen', 
            'fill_color': 'seagreen',
            'alpha': 0.5, 
            'line_width': 3
        } 
        
        
        
        polygon_params_pro = {
            'legend_label': 'dynamic_car expert',
            'line_color': 'chocolate', 
            'fill_color': 'chocolate',
            'alpha': 0.5, 
            'line_width': 3
        } 
        self.dynamic_car = fig.multi_polygons(
            self.polygon_xs, self.polygon_ys,
            source = self.polygon_datasource,
            **polygon_params
        )
        
        self.dynamic_car_pro = fig.multi_polygons(
            self.polygon_xs, self.polygon_ys,
            source = self.polygon_datasource_pro,
            **polygon_params_pro
        )
        #self.dynamic_car_pro.visible = False
        
        self.car_slider = Slider(start = 0, end = 1, value = 0, step = 1, title = "move nothing")
        self.car_slider_pro = Slider(start = 0, end = 1, value = 0, step = 1, title = "move car pro")
    
        self.hor_div = Div(width=400, height=20, height_policy="fixed")
        self.ver_div = Div(width=20, height=40, height_policy="fixed")
    
    def getLayout(self):
        return row(column(self.hor_div, self.car_slider,self.car_slider_pro, self.fig), self.ver_div)
    
    def update(self, odo, sbp_res_pro, sbp_res, car_size_params):    
        self.in_layer.updateUtil((odo, car_size_params))
        if sbp_res is not None:
            self.out_layer.updateUtil((sbp_res, car_size_params))
            self.addDynamicSlider(sbp_res, car_size_params)
            self.environment_slot_layer.updateUtil(sbp_res)
        
        if sbp_res_pro is not None:
            self.out_layer_pro.updateUtil((sbp_res_pro, car_size_params))
            self.addDynamicSliderPro(sbp_res_pro, car_size_params)
        
        
    def addDynamicSlider(self, sbp_res, car_size_params):
        steps = len(sbp_res.x)
        if steps ==0:
            return
        pts_xs = sbp_res.x
        pts_ys = sbp_res.y
        pts_thetas = sbp_res.phi
        
        car_xs=[]
        car_ys=[]
        for i in range(len(pts_xs)-1, -1, -1):
            carxx, caryy = getCarCorner(pts_xs[i], pts_ys[i], pts_thetas[i], car_size_params)     
            car_xs.append([[[carxx]]])
            car_ys.append([[[caryy]]])
        
        self.polygon_datasource.data.update({
            self.polygon_xs: car_xs[0],
            self.polygon_ys: car_ys[0],
            self.polygon_xs_array: [car_xs],
            self.polygon_ys_array: [car_ys],
        }) 
        # print(self.polygon_datasource.data)
        
        end_num = 0
        if steps > 0:
            end_num = steps - 1
        
        self.car_slider = Slider(start = 0, end = end_num, value = 0, step = 1, title = "move car")
        
        callback = CustomJS(args=dict(polygon_datasource = self.polygon_datasource),
                            code = """
    const step = cb_obj.value;
    const data = polygon_datasource.data;
    polygon_datasource.data['pts_xs'] = data['pts_xs_array'][0][step];
    polygon_datasource.data['pts_ys'] = data['pts_ys_array'][0][step];
    polygon_datasource.change.emit();
                            """)
        self.car_slider.js_on_change('value', callback)
    
    def addDynamicSliderPro(self, sbp_res, car_size_params):
        steps = len(sbp_res.x)
        if steps ==0:
            return
        pts_xs = sbp_res.x
        pts_ys = sbp_res.y
        pts_thetas = sbp_res.phi
        
        car_xs=[]
        car_ys=[]
        for i in range(len(pts_xs)-1, -1, -1):
            carxx, caryy = getCarCorner(pts_xs[i], pts_ys[i], pts_thetas[i], car_size_params)     
            car_xs.append([[[carxx]]])
            car_ys.append([[[caryy]]])
        
        self.polygon_datasource_pro.data.update({
            self.polygon_xs: car_xs[0],
            self.polygon_ys: car_ys[0],
            self.polygon_xs_array: [car_xs],
            self.polygon_ys_array: [car_ys],
        }) 
        # print(self.polygon_datasource.data)
        
        end_num = 0
        if steps > 0:
            end_num = steps - 1
        
        self.car_slider_pro = Slider(start = 0, end = end_num, value = 0, step = 1, title = "move car pro")
        callback = CustomJS(args=dict(polygon_datasource = self.polygon_datasource_pro),
                            code = """
    const step = cb_obj.value;
    const data = polygon_datasource.data;
    polygon_datasource.data['pts_xs'] = data['pts_xs_array'][0][step];
    polygon_datasource.data['pts_ys'] = data['pts_ys_array'][0][step];
    polygon_datasource.change.emit();
                            """)
        self.car_slider_pro.js_on_change('value', callback)


class PlannerFrameLayer:
    def __init__(self, fig) -> None:
        self.fig=fig
        self.in_layer = PlannerInputLayer(fig,[{},{},{},{},{},{},{'legend_label': 'target_car'}, {'legend_label': 'init_car'}])
        self.out_layer = PlannerOutputLayer(fig,  {'line_color': 'green', 'legend_label':'planner path'}, {'line_color': 'pink', 'legend_label':'car polygon'})
        self.environment_slot_layer = EnvironmentSlotLayer(fig,  {'line_color': 'blue', 'legend_label':'environment_slot'})        
        
        self.polygon_xs, self.polygon_ys = 'pts_xs', 'pts_ys'
        self.polygon_xs_array, self.polygon_ys_array = 'pts_xs_array', 'pts_ys_array'
        self.polygon_datasource = ColumnDataSource(data={
            self.polygon_xs: [],
            self.polygon_ys: [],
            self.polygon_xs_array: [],
            self.polygon_ys_array: []
        })
       
        polygon_params = {
            'legend_label': 'dynamic_car',
            'line_color': 'green', 
            'fill_color': 'green',
            'alpha': 0.5, 
            'line_width': 3
        } 
        
      
        self.dynamic_car = fig.multi_polygons(
            self.polygon_xs, self.polygon_ys,
            source = self.polygon_datasource,
            **polygon_params
        )
        
        points_param = {
            'legend_label': 'uss_points',
            'size': 3,
            'color': 'darkcyan'
        } 
        
        self.uss_layer = PointsLayer(fig, points_param)
        
               
        self.car_slider = Slider(start = 0, end = 1, value = 0, step = 1, title = "move nothing")
    
        self.hor_div = Div(width=400, height=20, height_policy="fixed")
        self.ver_div = Div(width=20, height=40, height_policy="fixed")
    
    def getLayout(self):
        return row(column(self.hor_div, self.car_slider,self.fig), self.ver_div)
    
    def update(self, odo,  sbp_res,uss_obs, car_size_params):    
        self.in_layer.updateUtil((odo, car_size_params))
        self.out_layer.updateUtil((sbp_res, car_size_params))
        # self.environment_slot_layer.updateUtil(sbp_res)
        self.addDynamicSlider(sbp_res, car_size_params)
        if uss_obs is not None:
            self.uss_layer.update(uss_obs[0], uss_obs[1])
        
        
    def addDynamicSlider(self, sbp_res, car_size_params):
        steps = len(sbp_res.x)
        if steps ==0:
            return
        pts_xs = sbp_res.x
        pts_ys = sbp_res.y
        pts_thetas = sbp_res.phi
        
        car_xs=[]
        car_ys=[]
        for i in range(len(pts_xs)-1, -1, -1):
            carxx, caryy = getCarCorner(pts_xs[i], pts_ys[i], pts_thetas[i], car_size_params)     
            car_xs.append([[[carxx]]])
            car_ys.append([[[caryy]]])
        
        self.polygon_datasource.data.update({
            self.polygon_xs: car_xs[0],
            self.polygon_ys: car_ys[0],
            self.polygon_xs_array: [car_xs],
            self.polygon_ys_array: [car_ys],
        }) 
        # print(self.polygon_datasource.data)
        
        end_num = 0
        if steps > 0:
            end_num = steps - 1
        
        self.car_slider = Slider(start = 0, end = end_num, value = 0, step = 1, title = "move car")
        
        callback = CustomJS(args=dict(polygon_datasource = self.polygon_datasource),
                            code = """
    const step = cb_obj.value;
    const data = polygon_datasource.data;
    polygon_datasource.data['pts_xs'] = data['pts_xs_array'][0][step];
    polygon_datasource.data['pts_ys'] = data['pts_ys_array'][0][step];
    polygon_datasource.change.emit();
                            """)
        self.car_slider.js_on_change('value', callback)
    
