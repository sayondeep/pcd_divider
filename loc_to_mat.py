import csv
import numpy as np
from pyntcloud import PyntCloud

# Load the .pcd file

def get_min_max():
    cloud = PyntCloud.from_file("/home/sayon/autoware_map/town01/pointcloud_map.pcd")

    # Retrieve the point cloud data
    points = cloud.points
    # print(points)
    # print(type(points))

    # Find the minimum and maximum coordinates
    min_x, min_y, min_z = points.min(axis=0)[:3]
    max_x, max_y, max_z = points.max(axis=0)[:3]

    # Calculate the grid size
    grid_size_x = max_x - min_x
    grid_size_y = max_y - min_y
    grid_size_z = max_z - min_z

    # Determine the total grid size

    # print(max_x)
    # print(min_x)
    # # print(grid_size_x)
    # print(max_y)
    # print(min_y)
    # print(grid_size_y)
    # total_grid_size = grid_size_x * grid_size_y * grid_size_z

    # print("Total grid size:", total_grid_size)
    return min_x,max_x,min_y,max_y

# Define the desired matrix size
import csv
import numpy as np

def save_binary_matrix(csv_filename, matrix_size, output_filename):
    try:
        # Read the CSV file and retrieve min/max x and y coordinates
        x_coords = []
        y_coords = []

        with open(csv_filename, "r") as csv_file:
            csv_reader = csv.DictReader(csv_file)

            if csv_reader.fieldnames != ["Vehicle ID", "X Coordinate", "Y Coordinate"]:
                raise ValueError("Invalid CSV file format.")

            for row in csv_reader:
                x = float(row["X Coordinate"])
                y = float(row["Y Coordinate"])
                x_coords.append(x)
                y_coords.append(y)

        # min_x = min(x_coords)
        # max_x = max(x_coords)
        # min_y = min(y_coords)
        # max_y = max(y_coords)

        min_x,max_x,min_y,max_y = get_min_max()

        print(min_x)
        print(max_x)
        
        # print(grid_size_x)
        print(min_y)
        print(max_y)
        

        # Scale the coordinates and create the binary matrix
        scaled_matrix = np.zeros((matrix_size, matrix_size), dtype=np.uint8)

        with open(csv_filename, "r") as csv_file:
            csv_reader = csv.DictReader(csv_file)

            for row in csv_reader:
                vehicle_id = row["Vehicle ID"]
                x = float(row["X Coordinate"])
                y = float(row["Y Coordinate"])

                # Scale the coordinates
                x_scaled = int((x - min_x) / (max_x - min_x) * (matrix_size - 1))
                # y_scaled = int(abs(y - min_y) / (max_y - min_y) * (matrix_size - 1))
                y_scaled = int(abs(y - max_y) / (max_y - min_y) * (matrix_size - 1))

                # Set the corresponding matrix cell to 1
                scaled_matrix[y_scaled, x_scaled] = 1

        # Save the scaled binary matrix to a text file
        with open(output_filename, "w") as output_file:
            # Write the dimensions of the matrix as the first row
            output_file.write(f"{matrix_size} {matrix_size}\n")

            # Write the binary matrix
            csv_writer = csv.writer(output_file)
            csv_writer.writerows(scaled_matrix)

        print("Matrix saved to", output_filename)

    except FileNotFoundError:
        print("CSV file not found.")
    except ValueError as e:
        print("Error:", str(e))

# Usage example
save_binary_matrix("vehicle_coordinates.csv", matrix_size=64, output_filename="matrix.txt")

