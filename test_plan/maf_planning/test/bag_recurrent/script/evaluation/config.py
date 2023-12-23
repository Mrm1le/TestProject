# -*- coding: utf-8 -*-

import os
import yaml

CONFIG_PATH = '{}/evaluation_config.yaml'.format(
    os.path.dirname(os.path.abspath(__file__)))

conf = dict()
with open(CONFIG_PATH, 'r') as f:
    conf = yaml.load(f, Loader=yaml.FullLoader)

    if 'extern_config' in conf['events'] and \
        isinstance(conf['events']['extern_config'], list):

        for extern_files in conf['events']['extern_config']:
            if not os.path.exists(extern_files):
                continue

            with open(extern_files, 'r') as fe:
                d = yaml.load(fe, Loader=yaml.FullLoader)

                for event_type, event_list in d.items():
                    if event_list == None:
                        continue

                    conf['events'][event_type].extend(event_list)

