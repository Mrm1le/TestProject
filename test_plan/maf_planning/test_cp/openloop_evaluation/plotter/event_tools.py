from plotter.basic_layers import MultiLinesLayer, TextLabelLayer, PointsLayer
from bokeh.plotting import ColumnDataSource
from bokeh.models.callbacks import CustomJS


class MeasureTools:
    def __init__(self, fig) -> None:
        self.fig = fig
        line_params = {
            'legend_label': 'measure tool',
            'alpha': 0.5,
            'line_width':4,
            'line_color': 'purple'
        }
        text_params = {}
        point_params = {
            'size': 6,
            'color': 'firebrick'
        }
        self.ends = PointsLayer(fig, point_params)
        self.line = MultiLinesLayer(fig, line_params)
        self.text = TextLabelLayer(fig, text_params)
        
        self.old_xs, self.old_ys = 'xs', 'ys'
        
        
        self.old_datasource = ColumnDataSource(data={
            self.old_xs: [],
            self.old_ys: []
        })
        
        self.setEvent()
    
    def setEvent(self):
        
        callback = CustomJS(
            args=dict(
                line_source =self.line.data_source, 
                text_source = self.text.data_source,
                end_source = self.ends.data_source,
                old_source = self.old_datasource
            ) ,
            code="""
const x=cb_obj.x;
const y=cb_obj.y;
console.log('Tap event occurred at x-position: ' + cb_obj.x)
console.log('Tap event occurred at y-position: ' + cb_obj.y)

if(old_source.data['xs'].length == 0){
    old_source.data['xs']=[x];
    old_source.data['ys']=[y];
    
    end_source.data['pts_xs']=[x];
    end_source.data['pts_ys']=[y];
    
    line_source.data['pts_xs']=[];
    line_source.data['pts_ys']=[];
    text_source.data['pts_xs']=[];
    text_source.data['pts_ys']=[];
    text_source.data['texts']=[];
}else{
    
    // get last point
    const ox = old_source.data['xs'][0];
    const oy = old_source.data['ys'][0];
    console.log(x,y,ox,oy);
    
    // clear last point
    old_source.data['xs']=[];
    old_source.data['ys']=[];
    
    // put last and current point
    end_source.data['pts_xs']=[x, ox];
    end_source.data['pts_ys']=[y, oy];
    line_source.data['pts_xs']=[[x, ox]];
    line_source.data['pts_ys']=[[y,oy]];
    text_source.data['pts_xs']=[(x+ox)/2.0];
    text_source.data['pts_ys']=[(y + oy)/2.0];
    
    const dis = Math.sqrt(Math.pow(ox-x,2) + Math.pow(oy-y,2));
    text_source.data['texts']=[dis.toFixed(4)];
    
}

line_source.change.emit();
text_source.change.emit();
old_source.change.emit();
end_source.change.emit();

""")
        self.fig.js_on_event('tap', callback)        
        