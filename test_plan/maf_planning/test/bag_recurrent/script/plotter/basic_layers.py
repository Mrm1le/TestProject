
from bokeh.plotting import ColumnDataSource
from bokeh.models import  LabelSet

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

class CircleLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys, self.rs = 'pts_xs', 'pts_ys', 'pts_rs'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.rs: []
    })

    self.plot = self.fig.circle(self.xs,
                  self.ys,
                  radius = self.rs,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, pts_rs):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
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
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.line(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })

class MultiLinesLayer:
  def __init__(self, fig, params):
    self.fig = fig
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
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })

class MultiPolygonLayer:
  def __init__(self, fig, params):
    self.fig = fig
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
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })

class MultiArcsLayer:
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
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.texts = 'texts'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.texts: []
    })
    
    self.plot = LabelSet(x=self.xs, y=self.ys, text=self.texts,
          x_offset=0, y_offset=0, source=self.data_source, **params)
    

    fig.add_layout(self.plot)
    
  def update(self, pts_xs, pts_ys, texts):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.texts: texts
    })

