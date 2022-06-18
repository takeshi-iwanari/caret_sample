import re
import numpy as np
import argparse
import networkx as nx
import dearpygui.dearpygui as dpg
from caret2networkx import caret2networkx
from networkx2dearpygui import networkx2dearpygui

def normalize_layout(layout):
    for key, val in layout.items():
        layout[key] = list(val)
    layout_np = np.array(list(layout.values()))
    layout_min, layout_max = layout_np.min(0), layout_np.max(0)
    for pos in layout.values():
        pos[0] -= layout_min[0]
        pos[0] /= (layout_max[0] - layout_min[0])
        pos[1] -= layout_min[1]
        pos[1] /= (layout_max[1] - layout_min[1])
        pos[1] = 1 - pos[1]
    return layout


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize Node Diagram using Architecture File Created by CARET')
    parser.add_argument('--architecture_yaml_file', type=str, default='architecture.yaml', help='Architecture (yaml) file path. default=architecture.yaml')
    parser.add_argument('--target_path', type=str, default='all', help='Specify path to be loaded. default=all')
    args = parser.parse_args()

    G = caret2networkx(args.architecture_yaml_file, args.target_path == 'all')

    # layout = nx.spring_layout(G)
    # layout = nx.nx_pydot.graphviz_layout(G, prog='dot')
    layout = nx.nx_pydot.pydot_layout(G, prog='dot')       # 'dot', 'twopi', 'fdp', 'sfdp', 'circo'
    layout = normalize_layout(layout)

    window_size = [1920, 1080]
    graph_size = window_size if len(G.nodes) < 20 else [n * 3 for n in window_size]

    # Draw the graph using Dear PyGui Node Editor
    dpg.create_context()
    networkx2dearpygui(G, layout, window_size[0], window_size[1], graph_size[0], graph_size[1])
    dpg.create_viewport(title='CARET Architecture Visualizer', width=graph_size[0], height=graph_size[1])
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()
