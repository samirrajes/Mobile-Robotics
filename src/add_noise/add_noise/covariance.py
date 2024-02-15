import numpy as np

def calculate_covariance_matrix(positions):
    """
    calculate the covariance matrix for a set of positions.

    parameters - positions: A list of tuples, where each tuple is a (x, y) position.

    returns covariance matrix for positions.
    """
    
    # convert tuple list into a numpy array
    positions_array = np.array(positions)
    
    # means of x and y
    x_mean = np.mean(positions_array[:, 0])
    y_mean = np.mean(positions_array[:, 1])
    
    # calculate covariance matrix
    covariance_matrix = np.cov(positions_array.T)  # Transpose to get the correct shape for np.cov
    
    return covariance_matrix

# Our data
exp1_positions = [(x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5), (x6, y6), (x7, y7), (x8, y8), (x9, y9), (x10, y10)]  # Experiment 1 positions
exp2_positions = [(x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5), (x6, y6), (x7, y7), (x8, y8), (x9, y9), (x10, y10)]  # Experiment 2 positions
exp3_positions = [(x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5), (x6, y6), (x7, y7), (x8, y8), (x9, y9), (x10, y10)]  # Experiment 3 positions

# Calculate covariance matrices
cov_matrix_exp1 = calculate_covariance_matrix(exp1_positions)
cov_matrix_exp2 = calculate_covariance_matrix(exp2_positions)
cov_matrix_exp3 = calculate_covariance_matrix(exp3_positions)

# Print the results
print("Covariance Matrix for Experiment 1:\n", cov_matrix_exp1)
print("Covariance Matrix for Experiment 2:\n", cov_matrix_exp2)
print("Covariance Matrix for Experiment 3:\n", cov_matrix_exp3)
