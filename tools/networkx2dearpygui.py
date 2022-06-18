from __future__ import annotations
import textwrap
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

    display_name = name.strip('"')
    display_name = textwrap.fill(display_name, 60)
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
    
    display_name = name.strip('"')
    display_name = textwrap.fill(display_name, 60)
    return display_name


class networkx2dearpygui:
    """
    Display node graph using Dear PyGui from NetworkX graph
    
    Attributes
    ----------
    G: nx.classes.digraph.DiGraph
        NetworkX Graph
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
    node_edge_dict: dict[str, tuple[list[str], list[str]]] = {} 
    dpg_node_id_dict: dict[str,int] = {}

    zoom_level: int = 10
    zoom_config: list = []
    font_list: dict[int, int] = {}

    def __init__(self, 
        app_setting: dict,
        G: nx.classes.digraph.DiGraph,
        window_width: int = 1920, window_height: int = 1080):
        """
        Parameters
        ----------
        app_setting: dict
            application settings
        G: nx.classes.digraph.DiGraph
            NetworkX Graph
        window_width : int,  default 1920
            Windows size
        window_height : int,  default 1080
            Windows size
        """

        self.G = G

        ''' Associate edge with node '''
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

        dpg.create_context()
        
        ''' Locate node and link '''
        with dpg.window(width=window_width, height=window_height, no_collapse=True, no_title_bar=True, no_move=True, no_resize=True) as self.window_id:
            with dpg.handler_registry():
                dpg.add_mouse_wheel_handler(callback=self.cb_wheel)
            self.make_zoom_table(app_setting['font'], window_width, window_height)

            with dpg.node_editor(menubar=False, minimap=True, minimap_location=dpg.mvNodeMiniMap_Location_BottomLeft) as self.nodeeditor_id:
                dpg_id_dict = {}    # {"nodename_edgename": id}

                ''' Add nodes '''
                for node_name in self.G.nodes:
                    pos = self.G.nodes[node_name]['pos']
                    pos = [pos[0] * self.zoom_config[self.zoom_level][1], pos[1] * self.zoom_config[self.zoom_level][2]]
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

                ''' Add links between nodes '''
                for edge in self.G.edges:
                    if 'label' in G.edges[edge]:
                        label = G.edges[edge]['label']
                        if (edge[1] + label in dpg_id_dict) and (edge[0] + label in dpg_id_dict):
                            edge_id = dpg.add_node_link(dpg_id_dict[edge[1] + label], dpg_id_dict[edge[0] + label])
                    else:
                        if (edge[1] + 'in' in dpg_id_dict) and (edge[0] + 'out' in dpg_id_dict):
                            edge_id = dpg.add_node_link(dpg_id_dict[edge[1] + 'in'], dpg_id_dict[edge[0] + 'out'])

                    ''' Link color is the same color as publisher '''
                    with dpg.theme() as theme_id:
                        with dpg.theme_component(dpg.mvNodeLink):
                            dpg.add_theme_color( dpg.mvNodeCol_Link, G.nodes[edge[0]]['color'], category = dpg.mvThemeCat_Nodes)
                        dpg.bind_item_theme(edge_id, theme_id)

        ''' Update node position and font according to the default zoom level '''
        self.cb_wheel(0, 0)

        ''' Dear PyGui stuffs '''
        dpg.create_viewport(title='CARET Architecture Visualizer', width=window_width, height=window_height)
        dpg.set_viewport_resize_callback(self.cb_resize)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.start_dearpygui()
        dpg.destroy_context()
    

    def cb_resize(self, sender, app_data):
        """
        callback function for window resized (Dear PyGui)
        change node editer size
        """    

        window_width = app_data[2]
        window_height = app_data[3]
        dpg.set_item_width(self.window_id, window_width)
        dpg.set_item_height(self.window_id, window_height)

    def cb_wheel(self, sender, app_data):
        """
        callback function for mouse wheel in node editor(Dear PyGui)
        zoom in/out graph according to wheel direction
        """    

        # Save current layout in normalized coordinate
        for node_name, node_id in self.dpg_node_id_dict.items():
            pos = dpg.get_item_pos(node_id)
            self.G.nodes[node_name]['pos'] = [pos[0] / self.zoom_config[self.zoom_level][1], pos[1] / self.zoom_config[self.zoom_level][2]]

        if app_data > 0:
            if self.zoom_level < len(self.zoom_config) - 1:
                self.zoom_level += 1
        elif app_data < 0:
            if self.zoom_level > 0:
                self.zoom_level -= 1


        # Update node position and font size according to new zoom level
        for node_name, node_id in self.dpg_node_id_dict.items():
            pos = self.G.nodes[node_name]['pos']
            pos[0], pos[1] = pos[0] * self.zoom_config[self.zoom_level][1], pos[1] * self.zoom_config[self.zoom_level][2]
            dpg.set_item_pos(node_id, pos)
        self.update_font()


    def update_font(self):
        """
        Update font used in all nodes according to current zoom level
        """
        for node_id in self.dpg_node_id_dict.values():
            dpg.bind_item_font(node_id, self.zoom_config[self.zoom_level][0])

    def make_zoom_table(self, font_path, window_width: int, window_height: int):
        with dpg.font_registry():
            for i in range(7, 20):
                try:
                    self.font_list[i] = dpg.add_font(font_path, i)
                except:
                    print('Failed to load font')
        self.zoom_config.append([self.font_list[10], window_width * 0.20, window_height * 0.20])
        self.zoom_config.append([self.font_list[10], window_width * 0.25, window_height * 0.25])
        self.zoom_config.append([self.font_list[11], window_width * 0.30, window_height * 0.30])
        self.zoom_level = len(self.zoom_config) - 1     # default zoom level
        self.zoom_config.append([self.font_list[11], window_width * 0.35, window_height * 0.35])
        self.zoom_config.append([self.font_list[12], window_width * 0.40, window_height * 0.40])
        self.zoom_config.append([self.font_list[12], window_width * 0.45, window_height * 0.45])
        self.zoom_config.append([self.font_list[13], window_width * 0.50, window_height * 0.50])
        self.zoom_config.append([self.font_list[13], window_width * 0.55, window_height * 0.55])
        self.zoom_config.append([self.font_list[14], window_width * 0.60, window_height * 0.60])
        self.zoom_config.append([self.font_list[14], window_width * 0.65, window_height * 0.65])
        self.zoom_config.append([self.font_list[14], window_width * 0.70, window_height * 0.70])
        self.zoom_config.append([self.font_list[15], window_width * 0.75, window_height * 0.75])
        self.zoom_config.append([self.font_list[15], window_width * 0.80, window_height * 0.80])
        self.zoom_config.append([self.font_list[15], window_width * 0.85, window_height * 0.85])
        self.zoom_config.append([self.font_list[16], window_width * 0.90, window_height * 0.90])
        self.zoom_config.append([self.font_list[16], window_width * 0.95, window_height * 0.95])
        self.zoom_config.append([self.font_list[16], window_width * 1.00, window_height * 1.00])
        self.zoom_config.append([self.font_list[17], window_width * 1.10, window_height * 1.10])
        self.zoom_config.append([self.font_list[17], window_width * 1.15, window_height * 1.15])
        self.zoom_config.append([self.font_list[17], window_width * 1.20, window_height * 1.20])
        self.zoom_config.append([self.font_list[17], window_width * 1.25, window_height * 1.25])
        self.zoom_config.append([self.font_list[17], window_width * 1.30, window_height * 1.30])
        self.zoom_config.append([self.font_list[17], window_width * 1.35, window_height * 1.35])
        self.zoom_config.append([self.font_list[18], window_width * 1.40, window_height * 1.40])
        self.zoom_config.append([self.font_list[18], window_width * 1.45, window_height * 1.45])
        self.zoom_config.append([self.font_list[18], window_width * 1.50, window_height * 1.50])
        self.zoom_config.append([self.font_list[18], window_width * 1.60, window_height * 1.60])


if __name__ == '__main__':
    G = nx.DiGraph()
    nx.add_path(G, ['3', '5', '4', '1', '0', '2'])
    nx.add_path(G, ['3', '0', '4', '2', '1', '5'])
    layout = nx.spring_layout(G)
    for key, val in layout.items():
        G.nodes[key]['pos'] = list(val)
    app_setting = {
        "font": "/usr/share/fonts/truetype/ubuntu/Ubuntu-C.ttf"
    }
    networkx2dearpygui(app_setting, G)

