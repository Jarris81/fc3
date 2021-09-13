import networkx as nx
import pydot
import matplotlib.pyplot as plt

def draw_search_graph(G, filename="action_tree.png"):

    pyg = nx.nx_pydot.to_pydot(G)

    # easier to build a new graph, and structure it accordingly
    graph = pydot.Dot(graph_type="digraph", rankdir="TB")

    edges = pyg.get_edges()

    for n in pyg.get_nodes():
        graph.add_node(n)

    for e in edges:
        graph.add_edge(e)

    for edge in pyg.get_edges():
        #edge.set("label", )
        action = edge.get_attributes()["action"]
        edge.set("label", f"{edge.get_source()} -> {edge.get_destination()} \n {action}")

    for node in graph.get_nodes():

        node.set("label", node.get_attributes()["state"])

    graph.write_png(filename)
    #pyg.write_png('output.png')
