import numpy as np

def apply_exponential_distribution(matrix, lambda_val):
    random_matrix = np.random.exponential(scale=1 / lambda_val, size=matrix.shape)
    return np.where(matrix == 1, np.where(random_matrix >= 1, 0, 1), matrix)

def save_matrix_to_txt(matrix, filename):
    with open(filename, 'w') as file:
        # Write the shape of the matrix as the first row
        file.write(f"{matrix.shape[0]} {matrix.shape[1]}\n")
        
        # Write the matrix entries
        for row in matrix:
            row_str = ' '.join(map(str, row))
            file.write(f"{row_str}\n")

def load_matrix_from_txt(filename):
    with open(filename, 'r') as file:
        # Read the shape of the matrix from the first row
        shape = file.readline().split()
        rows = int(shape[0])
        cols = int(shape[1])
        
        # Initialize an empty matrix
        matrix = np.zeros((rows, cols), dtype=int)
        
        # Read the matrix entries from the remaining lines
        for i, line in enumerate(file):
            row_entries = line.split()
            for j, entry in enumerate(row_entries):
                matrix[i, j] = int(entry)
        
        return matrix

# Example usage
filename = "matrix.txt"
lambda_val = 0.5

# Load the matrix from the file
matrix = load_matrix_from_txt(filename)

# Apply the exponential distribution
modified_matrix = apply_exponential_distribution(matrix, lambda_val)

# Save the modified matrix to a file
output_filename = "modified_matrix.txt"
save_matrix_to_txt(modified_matrix, output_filename)
print(f"Modified matrix saved to {output_filename}.")
