import ipywidgets


class OnOffButton(ipywidgets.Button):
  btn_style = {
    False: {
      'description': "Off",
      'icon': 'toggle-off',
    },
    True: {
      'description': 'On',
      'icon': 'toggle-on',
    }
  }

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.state = False  #OnOffButton default value(turn off)

  def on_click(self, func):
    def new_func(a):
      self.state = not self.state
      for k, v in self.btn_style[self.state].items():
        setattr(self, k, v)
      func(a)

    super().on_click(new_func)