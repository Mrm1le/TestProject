
from bokeh.plotting import ColumnDataSource
from bokeh.models import  LabelSet, HoverTool

class PointsLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.scatter(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })

class PointsLayerWithColors:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys, self.color = 'pts_xs', 'pts_ys', 'color'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.color: []
    })

    self.plot = self.fig.scatter(self.xs,
                  self.ys,
                  color = self.color, 
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, color):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.color: color
    })

class LanesLayer:
  def __init__(self, fig, params, reverse = False):
    self.fig = fig
    self.reverse = reverse
    if reverse:
      self.xs, self.ys = 'pts_ys', 'pts_xs'
    else:
      self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.multi_line(self.xs,
                                 self.ys,
                                 source=self.data_source,
                                 **params)


  def update(self, pts_xs, pts_ys):
    if not self.reverse:
      self.data_source.data.update({
        self.xs: pts_xs,
        self.ys: pts_ys,
      })
    else:
      self.data_source.data.update({
        self.xs: pts_ys,
        self.ys: pts_xs,
      })

class CircleLayer:
  def __init__(self, fig, params, reverse = False):
    self.fig = fig
    self.reverse = reverse
    if reverse:
      self.xs, self.ys = 'pts_ys', 'pts_xs'
    else:
      self.xs, self.ys = 'pts_xs', 'pts_ys'

    self.rs = 'pts_rs'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.rs: []
    })

    self.plot = self.fig.circle(x=self.xs,
                  y=self.ys,
                  radius = self.rs,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, pts_rs):
    if not self.reverse:
      self.data_source.data.update({
        self.xs: pts_xs,
        self.ys: pts_ys,
        self.rs: pts_rs
      })
    else:      
      self.data_source.data.update({
        self.xs: pts_ys,
        self.ys: pts_xs,
        self.rs: pts_rs
      })


class TriangleLayer:
  def __init__(self, fig, params, reverse = False):
    self.fig = fig
    self.reverse = reverse
    if reverse:
      self.xs, self.ys = 'pts_ys', 'pts_xs'
    else:
      self.xs, self.ys = 'pts_xs', 'pts_ys'

    self.rs = 'pts_rs'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.rs: []
    })

    self.plot = self.fig.triangle(x=self.xs,
                  y=self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, pts_rs):
    if not self.reverse:
      self.data_source.data.update({
        self.xs: pts_xs,
        self.ys: pts_ys,
        self.rs: pts_rs
      })
    else:      
      self.data_source.data.update({
        self.xs: pts_ys,
        self.ys: pts_xs,
        self.rs: pts_rs
      })


class MultiPolygonColorLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys, self.color = 'pts_xs', 'pts_ys', 'color'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.color: []
    })

    self.plot = self.fig.multi_polygons(self.xs,
                  self.ys,
                  color = self.color, 
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, color):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.color: color
    })
    
class CurveLayer:
    # 
  def __init__(self, fig, params, reverse = False):
    self.fig = fig
    self.reverse = reverse
    if reverse:
      self.xs, self.ys = 'pts_ys', 'pts_xs'
    else:
      self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })
    self.plot = self.fig.line(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)
    if params['legend_label'] == 'virtual lane'and self.reverse:
      hover1 = HoverTool(renderers = [self.plot],
                        tooltips=[('vl_x', '@pts_ys'),
                                  ('vl_y','@pts_xs')],
                        mode='vline')
      self.fig.add_tools(hover1)

    elif params['legend_label'] == 'clane'and self.reverse:
      hover1 = HoverTool(renderers = [self.plot],
                        tooltips=[('clane_x', '@pts_ys'),
                                  ('clane_y','@pts_xs')],
                        mode='hline')
      self.fig.add_tools(hover1)

    elif params['legend_label'] == 'planning_path'and self.reverse:
      hover1 = HoverTool(renderers = [self.plot],
                        tooltips=[('pp_x', '@pts_ys'),
                                  ('pp_y','@pts_xs')],
                        mode='hline',name='pp')
      self.fig.add_tools(hover1)

  def update(self, pts_xs, pts_ys):
    if not self.reverse:
      self.data_source.data.update({
        self.xs: pts_xs,
        self.ys: pts_ys,
      })
    else:
      self.data_source.data.update({
        self.xs: pts_ys,
        self.ys: pts_xs,
      })


class MultiLinesLayer:
  def __init__(self, fig, params, reverse = False):
    self.fig = fig
    self.reverse = reverse
    if reverse:
      self.xs, self.ys = 'pts_ys', 'pts_xs'
    else:
      self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.multi_line(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    if not self.reverse:
      self.data_source.data.update({
        self.xs: pts_xs,
        self.ys: pts_ys,
      })
    else:
      self.data_source.data.update({
        self.xs: pts_ys,
        self.ys: pts_xs,
      })

class MultiPolygonLayer:
  def __init__(self, fig, params, reverse = False):
    self.fig = fig
    self.reverse = reverse
    if reverse:
      self.xs, self.ys = 'pts_ys', 'pts_xs'
    else:
      self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.multi_polygons(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    if not self.reverse:
      self.data_source.data.update({
        self.xs: pts_xs,
        self.ys: pts_ys,
      })
    else:
      self.data_source.data.update({
        self.xs: pts_ys,
        self.ys: pts_xs,
      })

class MultiArcsLayer:
  def __init__(self, fig, params, reverse = False):
    self.fig = fig
    self.reverse = reverse
    if reverse:
      self.xs, self.ys = 'pts_ys', 'pts_xs'
    else:
      self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.rs, self.min_angle, self.max_angle = 'rs', 'min_angle', 'max_angle'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.rs: [],
      self.min_angle: [],
      self.max_angle: []
    })

    self.plot = self.fig.arc(
      self.xs,self.ys,
      radius = self.rs,
      start_angle = self.min_angle,
      end_angle = self.max_angle,
      source=self.data_source,
      **params)
  def update(self, pts_xs, pts_ys, rs, min_angle, max_angle):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.rs: rs,
      self.min_angle: min_angle,
      self.max_angle: max_angle
    })
class MultiWedgesLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.rs, self.min_angle, self.max_angle = 'rs', 'min_angle', 'max_angle'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.rs: [],
      self.min_angle: [],
      self.max_angle: []
    })

    self.plot = self.fig.wedge(
      self.xs,self.ys,
      radius = self.rs,
      start_angle = self.min_angle,
      end_angle = self.max_angle,
      source=self.data_source,
      **params)

  def update(self, pts_xs, pts_ys, rs, min_angle, max_angle):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.rs: rs,
      self.min_angle: min_angle,
      self.max_angle: max_angle
    })

class TextLabelLayer:
  def __init__(self, fig, params, reverse = False):
    self.fig = fig
    self.reverse = reverse
    if reverse:
      self.xs, self.ys = 'pts_ys', 'pts_xs'
    else:
      self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.texts = 'texts'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.texts: []
    })

    self.plot = LabelSet(x=self.xs, y=self.ys, text=self.texts,
          x_offset=0, y_offset=0, source=self.data_source, 
          text_font_size='12pt', 
          text_color = 'lightsteelblue', **params)
    

    fig.add_layout(self.plot)
    
  def update(self, pts_xs, pts_ys, texts):
    if not self.reverse:
      self.data_source.data.update({
        self.xs: pts_xs,
        self.ys: pts_ys,
        self.texts: texts
      })
    else:
      self.data_source.data.update({
        self.xs: pts_ys,
        self.ys: pts_xs,
        self.texts: texts
      })

