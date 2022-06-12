import argparse
import networkx as nx
import dearpygui.dearpygui as dpg
from caret2networkx import caret2networkx
from networkx2dearpygui_with_label import networkx2dearpygui

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize Node Diagram using Architecture File Created by CARET')
    parser.add_argument('--architecture_yaml_file', type=str, default='architecture.yaml', help='Architecture (yaml) file path. default=architecture.yaml')
    parser.add_argument('--target_path', type=str, default='all', help='Specify path to be loaded. default=all')
    args = parser.parse_args()
    load_all = (args.target_path == 'all')

    G = caret2networkx(args.architecture_yaml_file, load_all)

    # layout = nx.spring_layout(G)
    # layout = nx.nx_pydot.graphviz_layout(G, prog='dot')
    layout = nx.nx_pydot.pydot_layout(G, prog='dot')       # 'dot', 'twopi', 'fdp', 'sfdp', 'circo'
    for key, val in layout.items():
        layout[key] = list(val)

    # Draw the graph using Dear PyGui Node Editor
    dpg.create_context()
    networkx2dearpygui(G, layout, 1920, 1080, 1920 * 3 if load_all else 1920, 1080 * 3 if load_all else 1080)
    dpg.create_viewport(title='CARET Visualizer', width=1920, height=1080)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()
