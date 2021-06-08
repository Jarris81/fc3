import networkx as nx
import pydot
import matplotlib.pyplot as plt

def draw_search_graph(plan, state_plan, G, filename="action_tree.png"):
    #get all edges in plan

    #colors = ["r" if x in plan_id else "b" for x in G.nodes]
    #nx.draw_networkx(G, with_labels=True, node_color=colors)
    #plt.show()

    pyg = nx.nx_pydot.to_pydot(G)

    # easier to build a new graph, and structure it accordingly
    graph = pydot.Dot(graph_type="digraph", rankdir="TB")

    edges = pyg.get_edges()

    for n in pyg.get_nodes():
        graph.add_node(n)

    for e in edges:
        graph.add_edge(e)

    for a in plan:
        print(a)

    for s in state_plan:
        print(s, s.id)
        pyg.get_node(str(s.id))[0].set_shape("box")
        node = pydot.Node(str(s.id))
        # graph.add_node(node)
        #
        # if last_state:
        #     graph.add_edge(pydot.Edge(node, last_state))
        #
        # last_state = node

    for edge in pyg.get_edges():
        #edge.set("label", )
        action = edge.get_attributes()["action"]
        edge.set("label", f"{edge.get_source()} -> {edge.get_destination()} \n {action}")

    for node in graph.get_nodes():

        node.set("label", node.get_attributes()["state"])

    graph.write_png(filename)
    #pyg.write_png('output.png')
