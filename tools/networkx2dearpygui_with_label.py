import re
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import dearpygui.dearpygui as dpg

def replace_nodename(name):
    # name_list = re.split('[/, "]', name)
    # name_list = list(filter(None, name_list))
    # if len(name_list) > 1:
    #     name = '/' + name_list[0] + '\n/' + name_list[-1]
    # else:
    #     name = '/' + name_list[0]
    return name

def replace_topicname(name):
    # name_list = re.split('[/, "]', name)
    # name_list = list(filter(None, name_list))
    # name = '/' + name_list[-1]
    return name

def networkx2dearpygui(G, layout, window_width=1920, window_height=1080, graph_width=1920, graph_height=1080):
    # Get node and edge information
    node_list = G.nodes
    edge_list = G.edges

    # Associate edge with node
    node_edge_dict = {}  # {"node_name": [["/edge_out_name", ], ["/edge_in_name", ]]}
    for node in node_list:
        node_edge_dict[node] = [set([]), set([])]
    for edge in G.edges:
        if 'label' in G.edges[edge]:
            label = G.edges[edge]['label']
            node_edge_dict[edge[0]][0].add(label)
            node_edge_dict[edge[1]][1].add(label)
        else:
            node_edge_dict[edge[0]][0].add('out')
            node_edge_dict[edge[1]][1].add('in')

    # Convert layout ([0, 1.0] -> [0, width])
    for pos in layout.values():
        pos[0] *= graph_width
        pos[1] *= graph_height

    with dpg.font_registry():
        default_font = -1
        try:
            default_font = dpg.add_font("/usr/share/fonts/truetype/ubuntu/Ubuntu-C.ttf", 15)
        except:
            print('Failed to load font')

    with dpg.window(width=window_width, height=window_height):
        with dpg.node_editor(width=window_width, height=window_height):
            dpg_id_dict = {}    # {"nodename_edgename": id}

            for node_name in node_list:
                with dpg.node(label=replace_nodename(node_name), pos=layout[node_name]) as n_id:
                    if 'color' in G.nodes[node_name]:
                        with dpg.theme() as theme_id:
                            with dpg.theme_component(dpg.mvNode):
                                dpg.add_theme_color( dpg.mvNodeCol_TitleBar, G.nodes[node_name]['color'], category = dpg.mvThemeCat_Nodes)
                        dpg.bind_item_theme(n_id, theme_id)
                    if default_font != -1:
                        dpg.bind_item_font(n_id, default_font)
                    for edge_in in node_edge_dict[node_name][1]:
                        with dpg.node_attribute() as id:
                            dpg_id_dict[node_name + edge_in] = id
                            dpg.add_text(default_value=replace_topicname(edge_in))
                    for edge_out in node_edge_dict[node_name][0]:
                        with dpg.node_attribute(attribute_type=dpg.mvNode_Attr_Output) as id:
                            dpg_id_dict[node_name + edge_out] = id
                            dpg.add_text(default_value=replace_topicname(edge_out))

            for edge in edge_list:
                if 'label' in G.edges[edge]:
                    label = G.edges[edge]['label']
                    if (edge[1] + label in dpg_id_dict) and (edge[0] + label in dpg_id_dict):
                        dpg.add_node_link(dpg_id_dict[edge[1] + label], dpg_id_dict[edge[0] + label])
                else:
                    if (edge[1] + 'in' in dpg_id_dict) and (edge[0] + 'out' in dpg_id_dict):
                        dpg.add_node_link(dpg_id_dict[edge[1] + 'in'], dpg_id_dict[edge[0] + 'out'])


if __name__ == '__main__':
    # Create a graph
    G = nx.DiGraph()
    nx.add_path(G, ['3', '5', '4', '1', '0', '2'])
    nx.add_path(G, ['3', '0', '4', '2', '1', '5'])

    # layout = nx.spring_layout(G)
    # layout = nx.nx_pydot.graphviz_layout(G, prog='dot')
    layout = nx.nx_pydot.pydot_layout(G, prog='fdp')       # 'dot', 'twopi', 'fdp', 'sfdp', 'circo'
    for key, val in layout.items():
        layout[key] = list(val)

    # Draw the graph using Dear PyGui Node Editor
    dpg.create_context()
    networkx2dearpygui(G, layout)
    dpg.create_viewport(width=1920, height=1080)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()
