from __future__ import annotations
import re
import json
import numpy as np
import argparse
import networkx as nx
import dearpygui.dearpygui as dpg
from caret2networkx import caret2networkx
from networkx2dearpygui_with_label import networkx2dearpygui

def normalize_layout(layout: dict[str,tuple[int,int]]):
    """
    Normalize positions to [0.0, 1.0] (left-top = (0, 0))

    Parameters
    ----------
    layout: dict[str,tuple[int,int]]
        Dictionary of positions keyed by node.

    Returns
    -------
    layout: dict[str,tuple[int,int]]
        Dictionary of normalized positions keyed by node.
    """

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

def place_module(G: nx.classes.digraph.DiGraph, module_name: str, prog: str = 'dot'):
    """
    Place nodes belonging to module_name.
    Normalized position [x, y] is set to G.nodes[node]['pos']

    Parameters
    ----------
    G: nx.classes.digraph.DiGraph
        NetworkX Graph
    module_name: str
        module name
    prog: str  (default: 'dot')
        Name of the GraphViz command to use for layout.
        Options depend on GraphViz version but may include:
        'dot', 'twopi', 'fdp', 'sfdp', 'circo'

    Returns
    -------
    layout: dict[str,tuple[int,int]]
        Dictionary of normalized positions keyed by node.
    """

    H = nx.DiGraph()
    node_list = G.nodes
    for node in node_list:
        if module_name in node:
            H.add_node(node)
    edge_list = G.edges
    for edge in edge_list:
        if module_name in edge[0] and module_name in edge[1]:
            H.add_edge(edge[0], edge[1])
    layout_in_module = nx.nx_pydot.pydot_layout(H, prog=prog)
    layout_in_module = normalize_layout(layout_in_module)
    return layout_in_module


def place_all_groups(G, module_setting_filepath):
    """
    Quote name, because pydot requires. https://github.com/pydot/pydot/issues/258

    Parameters
    ----------
    name : str
        original name

    Returns
    -------
    modified_name : str
        name with '"'
    """

    with open(module_setting_filepath) as f:
        module_layout_offset = json.load(f)

    for node in G.nodes:
        G.nodes[node]['pos'] = [0, 0]

    for module_name, property in module_layout_offset.items():
        layout_in_module = place_module(G, module_name)
        offset = property['pos']
        color = property['color']

        for node in G.nodes:
            if module_name in node:
                pos = layout_in_module[node]
                G.nodes[node]['pos'] = [offset[0] + pos[0] * offset[2], offset[1] + pos[1] * offset[3]]
                G.nodes[node]['color'] = color

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize Node Diagram using Architecture File Created by CARET')
    parser.add_argument('--architecture_yaml_file', type=str, default='architecture.yaml', help='Architecture (yaml) file path. default=architecture.yaml')
    parser.add_argument('--target_path', type=str, default='all_graph', help='Specify path to be loaded. default=all_graph')
    parser.add_argument('--module_setting_file', type=str, default='module_setting.json', help='default=module_setting.json')
    args = parser.parse_args()

    G = caret2networkx(args.architecture_yaml_file, args.target_path)
    place_all_groups(G, args.module_setting_file)

    window_size = [1920, 1080]
    graph_size = [int(1920 * 0.8), int(1080 * 0.8)]

    # Draw the graph using Dear PyGui Node Editor
    dpg.create_context()
    networkx2dearpygui(G, window_size[0], window_size[1], graph_size[0], graph_size[1])
    dpg.create_viewport(title='CARET Architecture Visualizer', width=window_size[0], height=window_size[1])
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()
