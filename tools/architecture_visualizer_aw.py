import re
import json
import numpy as np
import argparse
import networkx as nx
import dearpygui.dearpygui as dpg
from caret2networkx import caret2networkx
from networkx2dearpygui_with_label import networkx2dearpygui

def normalize_layout(layout):
    if len(layout) == 0:
        return layout
    for key, val in layout.items():
        layout[key] = list(val)
    layout_np = np.array(list(layout.values()))
    layout_min, layout_max = layout_np.min(0), layout_np.max(0)
    norm_w = (layout_max[0] - layout_min[0])
    norm_h = (layout_max[1] - layout_min[1])
    if norm_w == 0 or norm_h == 0:
        return layout
    for pos in layout.values():
        pos[0] = (pos[0] - layout_min[0]) / norm_w
        pos[1] = (pos[1] - layout_min[1]) / norm_h
        pos[1] = 1 - pos[1]
    return layout

def allocate_module(G, module_name):
    H = nx.DiGraph()
    node_list = G.nodes
    for node in node_list:
        if module_name in node:
            H.add_node(node)
    edge_list = G.edges
    for edge in edge_list:
        if module_name in edge[0] and module_name in edge[1]:
            H.add_edge(edge[0], edge[1])
    layout_in_module = nx.nx_pydot.pydot_layout(H, prog='dot')       # 'dot', 'twopi', 'fdp', 'sfdp', 'circo'
    layout_in_module = normalize_layout(layout_in_module)
    return layout_in_module

def allocate_all_modules(G, module_setting_filepath):
    with open(module_setting_filepath) as f:
        module_layout_offset = json.load(f)

    layout = {}
    node_list = G.nodes
    for node in node_list:
        layout[node] = [0, 0]

    for module_name, property in module_layout_offset.items():
        layout_in_module = allocate_module(G, module_name)
        pos = property['pos']
        for val in layout_in_module.values():
            val[0], val[1] = pos[0] + val[0] * pos[2], pos[1] + val[1] * pos[3]
        layout.update(layout_in_module)

        node_list = G.nodes
        for node in node_list:
            if module_name in node:
                G.nodes[node]['color'] = property['color']
    
    return layout

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize Node Diagram using Architecture File Created by CARET')
    parser.add_argument('--architecture_yaml_file', type=str, default='architecture.yaml', help='Architecture (yaml) file path. default=architecture.yaml')
    parser.add_argument('--target_path', type=str, default='all', help='Specify path to be loaded. default=all')
    parser.add_argument('--module_setting_file', type=str, default='module_setting.json', help='default=module_setting.json')
    args = parser.parse_args()

    G = caret2networkx(args.architecture_yaml_file, args.target_path == 'all')

    layout = allocate_all_modules(G, args.module_setting_file)

    window_size = [1920, 1080]
    graph_size = [int(1920 * 0.8), int(1080 * 0.8)]

    # Draw the graph using Dear PyGui Node Editor
    dpg.create_context()
    networkx2dearpygui(G, layout, window_size[0], window_size[1], graph_size[0], graph_size[1])
    dpg.create_viewport(title='CARET Architecture Visualizer', width=window_size[0], height=window_size[1])
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()
