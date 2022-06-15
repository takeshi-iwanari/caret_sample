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

class networkx2dearpygui:
    window_width = 1920
    window_height = 1080
    graph_width = 1920
    graph_height = 1080
    layout_normalized = []
    G = []
    node_list = []
    edge_list = []
    node_edge_dict = {}  # {"node_name": [["/edge_out_name", ], ["/edge_in_name", ]]}
    dpg_node_id_list = []
    
    font_size = 15
    font_list = {}

    def __init__(self, G, layout_normalized, window_width=1920, window_height=1080, graph_width=1920, graph_height=1080):
        self.window_width = window_width
        self.window_height = window_height
        self.graph_width = graph_width
        self.graph_height = graph_height

        # Get node and edge information
        self.G = G
        self.node_list = G.nodes
        self.edge_list = G.edges
        self.layout_normalized = layout_normalized

        # Associate edge with node
        for node in self.node_list:
            self.node_edge_dict[node] = [set([]), set([])]
        for edge in G.edges:
            if 'label' in G.edges[edge]:
                label = G.edges[edge]['label']
                self.node_edge_dict[edge[0]][0].add(label)
                self.node_edge_dict[edge[1]][1].add(label)
            else:
                self.node_edge_dict[edge[0]][0].add('out')
                self.node_edge_dict[edge[1]][1].add('in')

        layout = {}
        for key, pos in self.layout_normalized.items():
            layout[key] = [pos[0] * self.graph_width, pos[1] * self.graph_height]

        with dpg.window(width=self.window_width, height=self.window_height):
            with dpg.node_editor(width=self.window_width, height=self.window_height):
                with dpg.handler_registry():
                    dpg.add_mouse_wheel_handler(callback=self.cb_wheel)
                dpg_id_dict = {}    # {"nodename_edgename": id}
                for node_name in self.node_list:
                    with dpg.node(label=replace_nodename(node_name), pos=layout[node_name]) as n_id:
                        self.dpg_node_id_list.append(n_id)
                        if 'color' in G.nodes[node_name]:
                            with dpg.theme() as theme_id:
                                with dpg.theme_component(dpg.mvNode):
                                    dpg.add_theme_color( dpg.mvNodeCol_TitleBar, G.nodes[node_name]['color'], category = dpg.mvThemeCat_Nodes)
                            dpg.bind_item_theme(n_id, theme_id)
                        for edge_in in self.node_edge_dict[node_name][1]:
                            with dpg.node_attribute() as id:
                                dpg_id_dict[node_name + edge_in] = id
                                dpg.add_text(default_value=replace_topicname(edge_in))
                        for edge_out in self.node_edge_dict[node_name][0]:
                            with dpg.node_attribute(attribute_type=dpg.mvNode_Attr_Output) as id:
                                dpg_id_dict[node_name + edge_out] = id
                                dpg.add_text(default_value=replace_topicname(edge_out))

                for edge in self.edge_list:
                    if 'label' in G.edges[edge]:
                        label = G.edges[edge]['label']
                        if (edge[1] + label in dpg_id_dict) and (edge[0] + label in dpg_id_dict):
                            dpg.add_node_link(dpg_id_dict[edge[1] + label], dpg_id_dict[edge[0] + label])
                    else:
                        if (edge[1] + 'in' in dpg_id_dict) and (edge[0] + 'out' in dpg_id_dict):
                            dpg.add_node_link(dpg_id_dict[edge[1] + 'in'], dpg_id_dict[edge[0] + 'out'])

        with dpg.font_registry():
            for i in range(7, 30):
                try:
                    self.font_list[i] = dpg.add_font("/usr/share/fonts/truetype/ubuntu/Ubuntu-C.ttf", i)
                except:
                    print('Failed to load font')

        self.update_font()


    def cb_wheel(self, sender, app_data):
        wheel = int(app_data)

        # Update current layout in normalized coordinate
        layout = []
        for node_id in self.dpg_node_id_list:
            pos = dpg.get_item_pos(node_id)
            layout.append([pos[0] / self.graph_width, pos[1] / self.graph_height])
        for index, key in enumerate(self.layout_normalized):
            self.layout_normalized[key] = layout[index]

        scale = 1.0
        if wheel > 0:
            if self.font_size < 20:
                self.font_size = self.font_size + 1
                scale = 1.1
        else:
            if self.font_size > 6:
                self.font_size = self.font_size - 1
                scale = 1.0 / 1.1
        self.graph_width *= scale
        self.graph_height *= scale

        layout = []
        for key, pos in self.layout_normalized.items():
            layout.append([pos[0] * self.graph_width, pos[1] * self.graph_height])

        for index, node_id in enumerate(self.dpg_node_id_list):
            dpg.set_item_pos(node_id, layout[index])

        self.update_font()


    def update_font(self):
        if self.font_size in self.font_list:
            for node_id in self.dpg_node_id_list:
                dpg.bind_item_font(node_id, self.font_list[self.font_size])

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
