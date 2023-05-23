import open3d as o3d
import numpy as np
import pandas as pd

class PointCloudHandler:
    def __init__(self, filepath):
        self.filepath = filepath
        self.XYZ_values = None

    def read_file(self):
        try:
            df = pd.read_csv(self.filepath)
            self.XYZ_values = np.array(df[['Point_X', 'Point_Y', 'Point_Z']])
            print("File {self.filepath} is found and no error.")

        except FileNotFoundError:
            print(f"Error: File '{self.filepath}' not found.")
        except pd.errors.EmptyDataError:
            print(f"Error: File '{self.filepath}' is empty.")
        except pd.errors.ParserError:
            print(f"Error: Unable to parse file '{self.filepath}'.")
        except KeyError:
            print(f"Error: Required columns 'Point_X', 'Point_Y', and 'Point_Z' not found in file '{self.filepath}'.")

    def visualize_pointcloud(self):
        self.read_file()
        if self.XYZ_values is None:
            print("Error: No point-cloud data to visualize.")
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.XYZ_values)
        o3d.visualization.draw_geometries([pcd], window_name='Visualizing the pointcloud', width=1920, height=1080, left=50, top=50, point_show_normal=True, mesh_show_wireframe=True, mesh_show_back_face=False)

    def select_wall(self):
        self.read_file()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.XYZ_values)

        # Fit a plane to the points in the point cloud using RANSAC to select the wall
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.045, ransac_n=3, num_iterations=10000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        #Extract the points that belong to the wall
        wall_points = pcd.select_by_index(inliers)
        non_wall_points = pcd.select_by_index(inliers, invert=True)

        #Visualize the point cloud with the wall points in blue and the non-wall points in red
        wall_points.paint_uniform_color([0.0, 0.0, 1.0])  # blue
        non_wall_points.paint_uniform_color([1.0, 0.0, 0.0])  # red

        o3d.visualization.draw_geometries([non_wall_points, wall_points], window_name='Blue = the wall points, Red = every other points', width=1920, height=1080, left=50, top=50, point_show_normal=True, mesh_show_wireframe=True, mesh_show_back_face=False)
        o3d.visualization.draw_geometries([wall_points], window_name='The separated wall', width=1920, height=1080, left=50, top=50, point_show_normal=True, mesh_show_wireframe=True, mesh_show_back_face=False)


    def measure_height_of_object(self):
        self.read_file()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.XYZ_values)
        vis = o3d.visualization.VisualizerWithVertexSelection()
        vis.create_window(window_name="Select the object (cylinder)")
        vis.add_geometry(pcd)
        vis.run()  # user picks points

        picked_points = vis.get_picked_points()
        y_coordinates = []
        x_coordinates = []
        for i in picked_points:
            y_coordinates.append(i.coord[2])
            x_coordinates.append(i.coord[0])
            print(i.index,i.coord)

        #print(y_coordinates)
        y_maxValue = max(y_coordinates)
        y_minValue = min(y_coordinates)
        print("From the selected points the max and min for measuring the height are:",y_maxValue,y_minValue)
        height_of_object = y_maxValue-y_minValue
        print("The height of the selected object is:",height_of_object)

        x_maxValue = max(x_coordinates)
        x_minValue = min(x_coordinates)
        print("From the selected points the max and min for measuring the diameter are:", x_maxValue, x_minValue)
        diameter_of_object = x_maxValue - x_minValue
        print(f"The diameter of the selected object is:'{diameter_of_object}' m.")

        vis.destroy_window()

filepath = 'DataFiles/CSV_file01.csv'

# Creating objects/instances:

file_checker = PointCloudHandler(filepath)
file_checker.read_file()

pch = PointCloudHandler(filepath)
pch.visualize_pointcloud()

wall = PointCloudHandler(filepath)
wall.select_wall()

selected_object = PointCloudHandler(filepath)
selected_object.measure_height_of_object()
