from abc import ABC, abstractmethod
from plotter.basic_layers import PointsLayer, CurveLayer, MultiLinesLayer, MultiPolygonLayer
from plotter.utils.common import convertBox, getCarCorner ,combineDict 


class EntityLayerBase(ABC):
    count = 0
    def __init__(self, geo_layer, fig, params) -> None:
        self.geo = geo_layer(fig, params)
        EntityLayerBase.count += 1
    
    def update(self, args):
        self.geo.update(*args)

    def updateUtil(self, data):
        data_converted = self.convert(data)
        self.update(data_converted)
    
    @abstractmethod
    def convert(self, data):
        pass


class ObsPoints(EntityLayerBase):
    def __init__(self, fig, params = None) -> None:
        fig_params = {
            'legend_label': 'obs_points',
            'size': 3,
            'color': 'firebrick'
        } 
        new_params = combineDict(fig_params, params)

        super().__init__(PointsLayer, fig, new_params)
        

    def convert(self, data):
        '''
        data: odo points array
        '''
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        for p in data:
            pts_xs.append(p['x_'])
            pts_ys.append(p['y_'])
        return (pts_xs, pts_ys)


class ObsLines(EntityLayerBase):
    def __init__(self, fig, params = None) -> None:

        fig_params = {
            'legend_label': 'obs_lines',
            'line_width': 1,
            'line_color': 'red'
        }
        new_params = combineDict(fig_params, params)
        super().__init__(MultiLinesLayer, fig, new_params)

    def convert(self, data):
        '''
        data: odo lines array
        '''
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        for line in data:
            pts_xs.append([line['start_']['x_'], line['end_']['x_']])
            pts_ys.append([line['start_']['y_'], line['end_']['y_']])
        return (pts_xs, pts_ys)    


class MapLines(ObsLines):
    def __init__(self, fig, params = None) -> None:

        fig_params = {
            'legend_label': 'map_lines',
            'line_width': 2,
            'line_color': 'red',
            'line_dash':'dashed'
        } 
        new_params = combineDict(fig_params, params)
        super().__init__(fig, new_params)

class PlannerPathLine(EntityLayerBase):
    def __init__(self, fig, params = None) -> None:
        fig_params = {
            'legend_label': 'planner_path_'+str(EntityLayerBase.count),
            'line_width': 3,
            'line_color': 'green'
        }
        new_params = combineDict(fig_params, params)

        super().__init__(CurveLayer, fig, new_params)

    def convert(self, data):
        '''
        data: odo lines array
        '''
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        for p in data:
            pts_xs.append(p['x'])
            pts_ys.append(p['y'])
        return (pts_xs, pts_ys) 

class SVLine(EntityLayerBase):
    def __init__(self, fig, params = None) -> None:
        fig_params = {
            'legend_label': 'sv_'+str(EntityLayerBase.count),
            'line_width': 3,
            'line_color': 'green'
        }
        new_params = combineDict(fig_params, params)

        super().__init__(CurveLayer, fig, new_params)

    def convert(self, data):
        '''
        data: odo lines array
        '''
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        for p in data:
            pts_xs.append(p[0])
            pts_ys.append(p[1])
        return (pts_xs, pts_ys) 


class MuiltiSVLines(EntityLayerBase):
    def __init__(self, fig, params = None) -> None:

        fig_params = {
            'legend_label': 'obs_lines',
            'line_width': 1,
            'line_color': 'red'
        }
        new_params = combineDict(fig_params, params)
        super().__init__(MultiLinesLayer, fig, new_params)

    def convert(self, data):
        '''
        data: odo lines array
        '''
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        for line in data:
            pts_xs.append([line['start_']['x_'], line['end_']['x_']])
            pts_ys.append([line['start_']['y_'], line['end_']['y_']])
        return (pts_xs, pts_ys)    

class SVLine(EntityLayerBase):
    def __init__(self, fig, params = None) -> None:
        fig_params = {
            'legend_label': 'sv_'+str(EntityLayerBase.count),
            'line_width': 3,
            'line_color': 'green'
        }
        new_params = combineDict(fig_params, params)

        super().__init__(CurveLayer, fig, new_params)

    def convert(self, data):
        '''
        data: odo lines array
        '''
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        for p in data:
            pts_xs.append(p[0])
            pts_ys.append(p[1])
        return (pts_xs, pts_ys) 

class EnvironmentSlotLine(EntityLayerBase):
    def __init__(self, fig, params = None) -> None:
        fig_params = {
            'legend_label': 'environment_slot',
            'line_width': 3,
            'line_color': 'blue',
            'line_dash':'dashed'
        }
        new_params = combineDict(fig_params, params)

        super().__init__(CurveLayer, fig, new_params)

    def convert(self, data):
        '''
        data: odo lines array
        '''
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        for p in data:
            pts_xs.append(p['x'])
            pts_ys.append(p['y'])
        return (pts_xs, pts_ys) 

class TLines(ObsLines):
    def __init__(self, fig, params = None) -> None:

        fig_params = {
            'legend_label': 't_lines',
            'line_width': 2,
            'line_color': 'grey',
            'line_dash':'dashed'
        } 
        new_params = combineDict(fig_params, params)
        super().__init__(fig, new_params)
    
    def convert(self, data):
        if data is None:
            return super().convert(data)

        lines=[
            data['road_upper_bound'],
            data['road_lower_left_bound'],
            data['slot_left_bound'],
            data['road_lower_right_bound'],
            data['slot_right_bound']
        ]

        return super().convert(lines)

class EgoCar(EntityLayerBase):
    num = 0
    def __init__(self,fig, params = None) -> None:
        self.num += 1
        fig_params = {
            'legend_label': 'ego_car_'+str(EntityLayerBase.count),
            'line_color': 'green', 
            'fill_color': 'green',
            'alpha': 0.5, 
            'line_width': 3
        } 
        new_params = combineDict(fig_params, params)

        super().__init__(MultiPolygonLayer, fig, new_params)
    
    def convert(self, data):
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        pose, car_params = data

        x = pose['x']
        y = pose['y']
        theta = pose['theta']
        carxx, caryy = getCarCorner(x, y, theta, car_params)
        
        pts_xs.append([[carxx]])
        pts_ys.append([[caryy]])
        return (pts_xs, pts_ys)  

class PlannerBox(EntityLayerBase):
    def __init__(self,fig, params = None) -> None:
        fig_params = {
            'legend_label': 'planner_box',
            'line_color': 'pink', 
            'fill_color': None, 
            'line_width': 2
        }
        new_params = combineDict(fig_params, params)

        super().__init__(MultiPolygonLayer, fig, new_params)
        self.geo.plot.visible = False
    
    def convert(self, data):
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys)
        poses, car_params = data
        
        for pose in poses:
            x = pose['x']
            y = pose['y']
            theta = pose['theta']
            carxx, caryy = getCarCorner(x, y, theta, car_params)     
            pts_xs.append([[carxx]])
            pts_ys.append([[caryy]])
        return (pts_xs, pts_ys)


class ObsBoxes(EntityLayerBase):
    def __init__(self,fig, params = None) -> None:
        fig_params = {
            'legend_label': 'obs_boxes',
            'line_color': 'red', 
            'fill_color': 'red', 
            'line_width': 2,
            'alpha':0.5
        } 
        new_params = combineDict(fig_params, params)

        super().__init__(MultiPolygonLayer, fig, new_params)
    
    def convert(self, data):
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 
        
        for box in data:
            x, y, theta, length, width = box['center_']['x_'], box['center_']['y_'], box['heading_'], box['length_'], box['width_']
            xs, ys = convertBox(x, y, theta, length, width)
            pts_xs.append([[xs]])
            pts_ys.append([[ys]])

        return (pts_xs, pts_ys)   

class SpeedMarginBoxes(EntityLayerBase):
    def __init__(self,fig, params = None) -> None:
        fig_params = {
            'legend_label': 'obs_boxes',
            'line_color': 'red', 
            'fill_color': None, 
            'line_width': 2,
            'alpha':0.5
        } 
        new_params = combineDict(fig_params, params)

        super().__init__(MultiPolygonLayer, fig, new_params)
    
    def convert(self, data):
        pts_xs = []
        pts_ys = []

        if data is None:
            return (pts_xs, pts_ys) 

        return data  

class PlannerBoundary(ObsBoxes):
    def __init__(self, fig, params=None) -> None:
        fig_params = {
            'legend_label': 'planner_boundary',
            'line_color': 'green', 
            'fill_color': None, 
            'line_width': 2
        }
        new_params = combineDict(fig_params, params)
        super().__init__(fig, params=new_params)
    
    def convert(self, data):
        if data is None:
            return super().convert(data)
        
        return super().convert([data])






