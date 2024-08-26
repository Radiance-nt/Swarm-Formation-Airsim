import random
from copy import deepcopy

from matplotlib import pyplot as plt

import binvox_rw
import numpy as np
import open3d as o3d
from scipy.ndimage import binary_dilation
import heapq
import math


def visualize_pcl(pcl):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Extract X, Y, and Z coordinates
    X = pcl[:, 0]
    Y = pcl[:, 1]
    Z = pcl[:, 2]
    # Create the scatter plot
    ax.scatter(X, Y, Z, s=1)  # Adjust the 's' parameter to control point size
    # Customize labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Point Cloud Visualization')
    # Show the plot
    plt.show()


def binvox_to_point_cloud(model):
    # 获取Vox模型的维度
    # dims = model.dims
    dims = model.data.shape
    # 创建一个空的点云对象
    point_cloud = o3d.geometry.PointCloud()

    # 遍历Vox模型的体素，并将非空体素的坐标添加到点云中
    points = []
    for x in range(dims[0]):
        for y in range(dims[1]):
            for z in range(dims[2]):
                if model.data[x, y, z]:
                    # 计算体素的实际坐标
                    point = [(x * 1.0), (y * 1.0), (z * 1.0)]  # 使用1.0作为体素的尺寸
                    points.append(point)

    # for i in range(10):
    #     min_z = min(point[2] for point in points)
    #     points = [point for point in points if point[2] != min_z]

    # 将点云数据设置为点的坐标
    point_cloud.points = o3d.utility.Vector3dVector(np.array(points))

    return point_cloud


# Define a function to calculate the Euclidean distance between two points
def euclidean_distance(point1, point2):
    return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2)))


def get_min_positive_index(arr):
    positive_indices = np.where(arr.astype(int) > 0)

    if positive_indices[0].size > 0:
        max_positive_index = np.argmin(arr[positive_indices])
        return tuple(positive_indices[i][max_positive_index] for i in range(len(positive_indices)))
    else:
        # 如果没有大于0的元素，返回None或其他适当的值
        return 0


def astar_search(start, goal, grid, search_step=2, diff=4, upward_height=5):
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.cla()
    # ax.scatter(start[0], start[1], start[2], c='g', s=40)
    # ax.scatter(goal[0], goal[1], goal[2], c='r', s=40)
    #
    # def update_plot(draw_set):
    #     for p in draw_set:
    #         ax.scatter(p[0], p[1], p[2], c='y', s=5)
    #     plt.pause(0.0001)

    open_set = []
    closed_set = set()
    came_from = {}

    g_score = {start: 0}
    f_score = {start: euclidean_distance(start, goal)}
    heapq.heappush(open_set, (f_score[start], start))
    t = 0

    while open_set:

        current = heapq.heappop(open_set)[1]

        if (
                abs(current[0] - goal[0]) <= diff
                and abs(current[1] - goal[1]) <= diff
                and abs(current[2] - goal[2]) <= diff
        ):
            return reconstruct_path(came_from, current)

        closed_set.add(current)
        t += 1
        draw_set = set()
        closed_set.add(current)
        draw_set.add(current)
        # if t % 20 == 0:
        # update_plot(draw_set)
        # draw_set.clear()

        for neighbor in get_neighbors(current, grid, closed_set, search_step,
                                      exceptions=[start, goal], upward_height=upward_height):

            tentative_g_score = g_score[current] + euclidean_distance(current, neighbor)

            if neighbor in closed_set and tentative_g_score >= g_score[neighbor]:
                continue

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + euclidean_distance(neighbor, goal)
                if neighbor not in open_set:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # No path found


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


def euclidean_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2) ** 0.5


def get_neighbors(point, grid, closed_set, search_step, exceptions=[], upward_height=0):
    candidates = []
    x, y, z = point
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            for k in (-1, 0, 1):
                if (i, j, k) != (0, 0, 0):
                    new_x = x + i * search_step
                    new_y = y + j * search_step
                    new_z = z + k * search_step
                    candidates.append((new_x, new_y, new_z))
    neighbors = []
    for candidate in candidates:
        if candidate not in closed_set and is_valid(candidate, grid, exceptions, upward_height):
            neighbors.append(candidate)

    return neighbors


# def is_valid(point, grid, exceptions=[], height=8):
#     x, y, z = point

#     if not (x >= 0 and x < grid.shape[0] and
#             y >= 0 and y < grid.shape[1] and
#             z >= 0 and z < grid.shape[2] + height + 1 and
#             grid[x, y, min(grid.shape[2] - 1, z)] == 0):
#         return False

#     for exception in exceptions:
#         if abs(x - exception[0]) > 4 or abs(y - exception[1]) > 4:
#             if grid[x, y, min(grid.shape[2] - 1, max(0, min(grid.shape[2] - height, z))):z].any():
#                 return False
#     return True

def is_valid(point, grid, exceptions=[], upward_height=0):
    x, y, z = point

    # 确保点在网格内
    if not (0 <= x < grid.shape[0] and 0 <= y < grid.shape[1] and 0 <= z < grid.shape[2]):
        return False

    # 检查点是否在例外区域内（即起点或终点附近）
    in_exception_area = False
    for exception in exceptions:
        if abs(x - exception[0]) <= 4 and abs(y - exception[1]) <= 4:
            in_exception_area = True
            break

    # 如果不在例外区域内，则检查高度是否有障碍物
    if not in_exception_area:
        for height_check in range(max(0, z - upward_height), z):
            if grid[x, y, min(grid.shape[2] - 1, height_check)] != 0:
                return False

    return grid[x, y, min(grid.shape[2] - 1, z)] == 0


class MapServer:
    def __init__(self, model, center=(0, 0, 0), res=1, inflation_radius=1, visualize_once=False,
                 generate_mesh=False, voxel_size=4, map_size=(0, 0, 0)):
        self.vis = None
        self.model = model
        self.center = center
        self.res = res
        if model is not None:
            self.scale = self.shape
        else:
            self.scale = map_size
        if model is None:
            return
        self.point_cloud = binvox_to_point_cloud(self.model)
        # Define the voxel size for downsampling
        if voxel_size != 0:
            self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size=voxel_size)
        alpha = 10  # parameter for alpha_shape
        self.point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10, max_nn=30))
        vis_list = [self.point_cloud]
        self.mesh = None
        if generate_mesh:
            self.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(self.point_cloud, alpha)
            self.mesh.compute_vertex_normals()
            vis_list.append(self.mesh)
        if visualize_once:
            o3d.visualization.draw_geometries(vis_list)

        def inflate_obstacles(binvox_data, inflation_radius):
            struct = np.ones((2 * inflation_radius + 1,) * 3, dtype=bool)
            return binary_dilation(binvox_data, structure=struct, iterations=1).astype(np.uint8)

        self.inflated_binvox_model = None
        if inflation_radius:
            self.inflated_binvox_model = inflate_obstacles(self.model.data, inflation_radius)
        else:
            self.inflated_binvox_model = self.model.data
            # inflated_point_cloud = binvox_to_point_cloud(inflated_binvox_model)

    def visualize_path(self, path, start=None, end=None):
        # Adding start and end to the path if they are provided
        path = deepcopy(path)
        if start is not None:
            path.insert(0, start)
        if end is not None:
            path.append(end)

        path_points_array = np.array(path)

        # Create a point cloud for the path points
        path_point_cloud = o3d.geometry.PointCloud()
        path_point_cloud.points = o3d.utility.Vector3dVector(path_points_array)

        # Create a LineSet object
        path_lines = o3d.geometry.LineSet()

        # Define line segments
        lines = [[i, i + 1] for i in range(len(path) - 1)]
        path_lines.lines = o3d.utility.Vector2iVector(lines)
        path_lines.points = o3d.utility.Vector3dVector(path_points_array)

        # Create spheres for start and end points if they exist
        # if self.vis is None:
        #     vis_list = [self.point_cloud]
        #     from open3d._ml3d.vis import Visualizer
        #     self.vis = o3d.visualization.Visualizer()
        #     self.vis.create_window()
        # else:
        #     vis_list = []
        vis_list = [self.point_cloud]
        vis_list += [path_point_cloud, path_lines]
        if start is not None:
            start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=3)
            start_sphere.translate(start)
            vis_list.append(start_sphere)
        if end is not None:
            end_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=5)
            end_sphere.translate(end)
            vis_list.append(end_sphere)

        # Add mesh if it exists
        if self.mesh is not None:
            vis_list.append(self.mesh)

        # Visualize
        o3d.visualization.draw_geometries(vis_list)
        # self.vis.update_geometry(vis_list)
        # self.vis.poll_events()
        # self.vis.update_renderer()

    def navigate(self, start_point, goal_point):

        search_model = self.inflated_binvox_model if self.inflated_binvox_model is not None else self.model
        path = astar_search(start_point, goal_point, search_model,
                            search_step=2, diff=3, upward_height=10)

        # path = RRT(start_point, goal_point, self.point_cloud, mesh=self.mesh)

        return path

    def is_occupy(self, point, clearance_radius):
        grid_shape = self.shape
        x, y, z = point
        is_clear = True
        k = 0
        for i in range(-clearance_radius, clearance_radius + 1):
            for j in range(-clearance_radius, clearance_radius + 1):
                if not (0 <= x + i < grid_shape[0] and 0 <= y + j < grid_shape[1] and 0 <= z + k < grid_shape[2]):
                    continue
                if (
                        0 <= x + i < grid_shape[0]
                        and 0 <= y + j < grid_shape[1]
                        and self.model.data[x + i, y + j, z] != 0
                ):
                    is_clear = False
                    return True
            if not is_clear:
                return True

        return False

    @property
    def shape(self):
        return self.model.data.shape

    def coord_to_index(self, position):
        x, y, z = position
        z = -z
        x += self.shape[0] / 2 - self.center[0]
        y += self.shape[1] / 2 - self.center[0]
        z += self.shape[2] / 2 - self.center[0]
        x, y, z = int(y), int(x), int(z)
        # Floor
        z += 2
        return (x, y, z)

    def index_to_coord(self, point):
        x, y, z = point
        # Floor

        z -= 2
        x += self.center[0] - self.shape[0] / 2
        y += self.center[1] - self.shape[1] / 2
        z += self.center[2] - self.shape[2] / 2
        x, y, z = float(y), float(x), -float(z)
        return (x, y, z)


if __name__ == "__main__":
    binvox_file = "map_1.binvox"  # 替换为你的二进制Vox文件的路径

    with open(binvox_file, 'rb') as f:
        model = binvox_rw.read_as_3d_array(f)

    map_server = MapServer(model)

    uav_position = (200.0, 200.0, 30.0)  # 无人机的当前位置，表示为元组

    grid_shape = map_server.shape
    print(grid_shape)
    while True:
        # Generate random XY coordinates
        x = random.randint(0, grid_shape[0] - 1)
        y = random.randint(0, grid_shape[1] - 1)
        z = 30
        if not map_server.is_occupy((x, y, z), 5):
            goal_position = (x, y, z)
            break

    start_point = tuple(map(int, uav_position))
    goal_point = tuple(map(int, goal_position))
    print(start_point)
    print(goal_point)
    path = map_server.navigate(start_point, goal_point)

    map_server.visualize_path(path, start_point, goal_point)
