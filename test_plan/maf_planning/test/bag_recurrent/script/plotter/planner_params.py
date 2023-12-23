import ipywidgets

from collections import namedtuple

ParamsBody = namedtuple('ParamsBody', [ 'label', 'type', 'default_value', 'min', 'max', 'step'])
TUNED_PARAMS = {
    'lon_inflation':ParamsBody('lon_inflation', 'slider', 0.4, 0.2, 0.7, 0.01),
    'next_node_num':ParamsBody('next_node_num', 'slider', 13, 3, 13, 2),
    'lat_inflation':ParamsBody('lat_inflation', 'slider', 0.22, 0.0, 0.4, 0.01),
    'traj_gear_switch_penalty':ParamsBody('traj_gear_switch_penalty', 'slider', 5.0, 1.0, 10.0, 1.0),
    'max_zigzag_allowed':ParamsBody('max_zigzag_allowed', 'slider', 5, 3, 20, 1),
    'xy_grid_resolution':ParamsBody('xy_grid_resolution', 'slider', 0.3, 0.1, 0.5, 0.1),
    'step_size':ParamsBody('step_size', 'slider', 0.5, 0.1, 0.5, 0.1),
    'max_iter':ParamsBody('max_iter', 'slider', 5000, 5000, 30000, 5000),
    'planning_core': ParamsBody('planning_core', 'slider', 0, 0, 20, 1),
    'max_iter_base': ParamsBody('max_iter_base', 'slider', 150000, 10000, 1000000, 10000),
    'max_iter_max': ParamsBody('max_iter_max', 'slider', 500000, 10000, 1000000, 10000),
    'traj_steer_change_penalty_gear_switch': ParamsBody('traj_steer_change_penalty_gear_switch', 'slider', 0.05, 0.0, 1, 0.01),
    'traj_obstacle_distance_1_penalty': ParamsBody('traj_obstacle_distance_1_penalty', 'slider', 5.0, 0.0, 100.0, 1.0),
    'traj_obstacle_distance_2_penalty': ParamsBody('traj_obstacle_distance_2_penalty', 'slider', 10.0, 0.0, 100.0, 1.0),
    'traj_obstacle_distance_3_penalty': ParamsBody('traj_obstacle_distance_3_penalty', 'slider', 20.0, 0.0, 100.0, 1.0),
    'traj_end_offset_penalty': ParamsBody('traj_end_offset_penalty', 'slider', 20.0, 0.0, 100.0, 1.0),
    'traj_s_turn_penalty': ParamsBody('traj_s_turn_penalty', 'slider', 5.0, 0.0, 20.0, 0.2),
}

class SliderParams():
    def __init__(self, label,value, min, max, step) -> None:
        self.value = value
        self.label = label

        self.slider = self.sliderOption(self.value)(
            layout = ipywidgets.Layout(width="60%"),
            value = self.value,
            min = min,
            max = max,
            step = step,
            stype={
                'description_width': 'auto'
            }
        )
    
    def setCallBack(self, f):
        self.slider.observe(f, names=['value'])
    
    @staticmethod
    def sliderOption(val):
        if isinstance(val, float):
            return ipywidgets.FloatSlider
        elif isinstance(val, int):
            return ipywidgets.IntSlider
        else:
            raise ValueError(
                "{} not supported, only support: float & int".format(val))

class TextParams():
    def __init__(self, label,value) -> None:
        self.value = value
        self.label = label

        self.slider = self.textOption(self.value)(
            layout = ipywidgets.Layout(width="40%"),
            value = self.value,
            description=self.label,
            stype={
                'description_width': 'auto'
            }
        )
    def setCallBack(self, f):
        self.slider.observe(f, names=['value'])

    @staticmethod
    def textOption(val):
        if isinstance(val, float):
            return ipywidgets.FloatText
        elif isinstance(val, int):
            return ipywidgets.IntText
        else:
            raise ValueError(
                "{} not supported, only support: float & int".format(val))

    

def getParamWidget(pb):
    if pb.type == 'slider':
        return SliderParams(pb.label, pb.default_value, pb.min, pb.max, pb.step)
    elif pb.type == 'text':
        return TextParams(pb.label, pb.default_value)
    else:
        return None

        