import networkx as nx
import matplotlib.pyplot as plt
import yaml

def string_name(name):
    name = '"' + name + '"'
    return name

def caret2networkx(filename, load_all=True):
    node_name_list = []

    topic_pub_dict = {
        # "/topic_0": ["/node_0"],
    }

    topic_sub_dict = {
        # "/topic_0": ["/node_1"],
    }

    with open(filename) as file:
        yml = yaml.safe_load(file)
        if load_all:
            nodes = yml['nodes']
            for node in nodes:
                node_name = string_name(node['node_name'])
                node_name_list.append(node_name)
                if 'publishes' in node:
                    publishes = node['publishes']
                    for publish in publishes:
                        if publish['topic_name'] in topic_pub_dict:
                            topic_pub_dict[publish['topic_name']].append(node_name)
                        else:
                            topic_pub_dict[publish['topic_name']] = [node_name]
                if 'subscribes' in node:
                    subscribes = node['subscribes']
                    for subscribe in subscribes:
                        if subscribe['topic_name'] in topic_sub_dict:
                            topic_sub_dict[subscribe['topic_name']].append(node_name)
                        else:
                            topic_sub_dict[subscribe['topic_name']] = [node_name]
        else:
            named_paths = yml['named_paths']
            if len(named_paths) > 0:
                for named_path in named_paths:
                    node_chain = named_path['node_chain']
                    for node in node_chain:
                        node_name = string_name(node['node_name'])
                        if node['publish_topic_name'] != 'UNDEFINED':
                            topic_pub_dict[node['publish_topic_name']] = [node_name]
                        if node['subscribe_topic_name'] != 'UNDEFINED':
                            topic_sub_dict[node['subscribe_topic_name']] = [node_name]
            else:
                print('named_paths not found')

    G = nx.DiGraph()
    # G.add_nodes_from(node_name_list)

    for topic, node_pub_list in topic_pub_dict.items():
        if topic in topic_sub_dict:
            node_sub_list = topic_sub_dict[topic]
        else:
            # node_sub_list = ["none:" + topic]
            continue
        for node_pub in node_pub_list:
            for node_sub in node_sub_list:
                # print(topic, node_pub, node_sub)
                G.add_edge(node_pub, node_sub, label=string_name(topic))

    print('len(connected_nodes) = ' + str(len(G.nodes)) + ', len(nodes) = ' + str(len(node_name_list)))

    return G

if __name__ == '__main__':
    G = caret2networkx('architecture.yaml')
    pos = nx.spring_layout(G)
    # pos = nx.circular_layout(G)
    nx.draw_networkx(G, pos)
    plt.show()