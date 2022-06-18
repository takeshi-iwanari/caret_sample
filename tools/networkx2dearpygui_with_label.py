from __future__ import annotations
import re
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import dearpygui.dearpygui as dpg

def replace_nodename(name):
    """
    replace an original node name to a name to be displayed
    
    Parameters
    ----------
    name : str
        original node name

    Returns
    -------
    display_name : str
        name to be displayed
    """
    # name_list = re.split('[/, "]', name)
    # name_list = list(filter(None, name_list))
    # if len(name_list) > 1:
    #     display_name = '/' + name_list[0] + '\n/' + name_list[-1]
    # else:
    #     display_name = '/' + name_list[0]
    display_name = name
    return display_name

def replace_edgename(name):
    """
    replace an original edge name to a name to be displayed
    
    Parameters
    ----------
    name : str
        original node name

    Returns
    -------
    display_name : str
        name to be displayed
    """
    # name_list = re.split('[/, "]', name)
    # name_list = list(filter(None, name_list))
    # display_name = '/' + name_list[-1]
    display_name = name
    return display_name


class networkx2dearpygui:
    """
    Display node graph using Dear PyGui from NetworkX graph
    
    Attributes
    ----------
    G: nx.classes.digraph.DiGraph
        NetworkX Graph
    window_width : int
        Windows size
    window_height : int
        Windows size
    graph_width : int
        Graph size
    graph_height : int
        Graph size
    node_list: list[str]
        list of nodes in G
    edge_list: list[str]
        list of edges in G
    node_edge_dict: dict[str, tuple[list[str], list[str]]]
        association between nod eand edge
        {"node_name": [["/edge_out_name", ], ["/edge_in_name", ]]}
    dpg_node_id_dict: dict[str,int]
        association between node_name and dpg.node_id
    font_size: int
        current font size
    font_list: dict[int, int]
        association between font_size and dpg_font_id
    """

    G: nx.classes.digraph.DiGraph
    window_width: int = 1920
    window_height: int = 1080
    graph_width: int = 1920
    graph_height: int = 1080
    node_edge_dict: dict[str, tuple[list[str], list[str]]] = {} 
    dpg_node_id_dict: dict[str,int] = {}
    font_size: int = 15
    font_list: dict[int, int] = {}

    def __init__(self, 
        G: nx.classes.digraph.DiGraph,
        window_width: int = 1920, window_height: int = 1080, 
        graph_width: int = 1920, graph_height: int = 1080):
        """
        Parameters
        ----------
        G: nx.classes.digraph.DiGraph
            NetworkX Graph
        window_width : int,  default 1920
            Windows size
        window_height : int,  default 1080
            Windows size
        graph_width : int,  default 1920
            Graph size
        graph_height : int,  default 1080
            Graph size

        """

        self.G = G
        self.window_width = window_width
        self.window_height = window_height
        self.graph_width = graph_width
        self.graph_height = graph_height

        # Associate edge with node
        for node_name in self.G.nodes:
            self.node_edge_dict[node_name] = [set([]), set([])]
        for edge in G.edges:
            if 'label' in G.edges[edge]:
                label = G.edges[edge]['label']
                self.node_edge_dict[edge[0]][0].add(label)
                self.node_edge_dict[edge[1]][1].add(label)
            else:
                self.node_edge_dict[edge[0]][0].add('out')
                self.node_edge_dict[edge[1]][1].add('in')

        with dpg.window(width=self.window_width, height=self.window_height):
            with dpg.node_editor(width=self.window_width, height=self.window_height):
                with dpg.handler_registry():
                    dpg.add_mouse_wheel_handler(callback=self.cb_wheel)
                dpg_id_dict = {}    # {"nodename_edgename": id}
                for node_name in self.G.nodes:
                    pos = self.G.nodes[node_name]['pos']
                    pos = [pos[0] * self.graph_width, pos[1] * self.graph_height]

                    with dpg.node(label=replace_nodename(node_name), pos=pos) as node_id:
                        self.dpg_node_id_dict[node_name] = node_id
                        if 'color' in G.nodes[node_name]:
                            with dpg.theme() as theme_id:
                                with dpg.theme_component(dpg.mvNode):
                                    dpg.add_theme_color( dpg.mvNodeCol_TitleBar, G.nodes[node_name]['color'], category = dpg.mvThemeCat_Nodes)
                            dpg.bind_item_theme(node_id, theme_id)
                        for edge_in in self.node_edge_dict[node_name][1]:
                            with dpg.node_attribute() as id:
                                dpg_id_dict[node_name + edge_in] = id
                                dpg.add_text(default_value=replace_edgename(edge_in))
                        for edge_out in self.node_edge_dict[node_name][0]:
                            with dpg.node_attribute(attribute_type=dpg.mvNode_Attr_Output) as id:
                                dpg_id_dict[node_name + edge_out] = id
                                dpg.add_text(default_value=replace_edgename(edge_out))

                for edge in self.G.edges:
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
        """
        callback function for mouse wheel in node editor(Dear PyGui)
        zoom in/out graph according to wheel direction
        
        Parameters
        ----------
        sender : int
            see Dear PyGui document
        app_data : int
            see Dear PyGui document
        """    

        wheel = int(app_data)

        # Update current layout in normalized coordinate
        for node_name, node_id in self.dpg_node_id_dict.items():
            pos = dpg.get_item_pos(node_id)
            self.G.nodes[node_name]['pos'] = [pos[0] / self.graph_width, pos[1] / self.graph_height]

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

        for node_name, node_id in self.dpg_node_id_dict.items():
            pos = self.G.nodes[node_name]['pos']
            pos[0], pos[1] = pos[0] * self.graph_width, pos[1] * self.graph_height
            dpg.set_item_pos(node_id, pos)

        self.update_font()


    def update_font(self):
        """
        Update font used in all nodes according to current font_size
        """
        if self.font_size in self.font_list:
            for node_id in self.dpg_node_id_dict.values():
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
