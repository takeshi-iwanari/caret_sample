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

    # Convert layout ([-1, 1] -> [0, width])
    layout_np = np.array(list(layout.values()))
    layout_min, layout_max = layout_np.min(0), layout_np.max(0)
    for pos in layout.values():
        pos[0] -= layout_min[0]
        pos[0] /= (layout_max[0] - layout_min[0])
        pos[1] -= layout_min[1]
        pos[1] /= (layout_max[1] - layout_min[1])
        pos[1] = 1 - pos[1]
        pos[0] *= graph_width
        pos[1] *= graph_height

    with dpg.window(width=window_width, height=window_height):
        with dpg.node_editor(width=graph_width, height=graph_height):
            dpg_id_dict = {}    # {"nodename_edgename": id}
            for node_name in node_list:
                with dpg.node(label=replace_nodename(node_name), pos=layout[node_name]):
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
