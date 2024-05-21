# 导入所需的库和模块
import numpy as np
import laspy as lp
import networkx as nx

# 读取点云数据，并提取x, y, z, intensity四个属性
inFile = lp.file.File("points.las", mode="r")
points = inFile.points.copy()
x = inFile.x.copy()
y = inFile.y.copy()
z = inFile.z.copy()
intensity = inFile.intensity.copy()

# 构建一个无向图，用于存储点云之间的距离和权重
G = nx.Graph()
# 遍历所有的点，计算两两之间的欧氏距离，并将其作为边的权重
for i in range(len(points)):
    for j in range(i + 1, len(points)):
        dist = np.sqrt((x[i] - x[j]) ** 2 + (y[i] - y[j]) ** 2 + (z[i] - z[j]) ** 2)
        G.add_edge(i, j, weight=dist)

# 使用networkx提供的kruskal_mst函数，求解最小生成树
mst = nx.tree.kruskal_mst(G)
# 将最小生成树转换为一个子图，用于后续的操作
mst_graph = G.edge_subgraph(mst)

# 定义一个函数，用于计算单木的树高和胸径
def measure_tree(tree):
    # tree是一个子图，表示一棵单木
    # 树高等于子图中z值的最大值减去最小值
    tree_height = max(z[list(tree.nodes)]) - min(z[list(tree.nodes)])
    # 胸径等于子图中intensity值的平均值乘以一个系数（假设为0.01）
    tree_diameter = np.mean(intensity[list(tree.nodes)]) * 0.01
    return tree_height, tree_diameter

# 定义一个列表，用于存储每棵单木的树高和胸径
tree_measures = []
# 使用networkx提供的connected_components函数，将最小生成树划分为若干个连通分量，每个分量表示一棵单木
for component in nx.connected_components(mst_graph):
    # 提取每个分量对应的子图
    tree = mst_graph.subgraph(component)
    # 计算每棵单木的树高和胸径，并将其添加到列表中
    tree_height, tree_diameter = measure_tree(tree)
    tree_measures.append((tree_height, tree_diameter))

# 输出每棵单木的树高和胸径
print("Tree height and diameter for each tree:")
for i, measure in enumerate(tree_measures):
    print(f"Tree {i + 1}: height = {measure[0]:.2f}, diameter = {measure[1]:.2f}")

