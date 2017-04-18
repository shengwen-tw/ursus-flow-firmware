import numpy as np
import matplotlib.pyplot as plot
import scipy.ndimage.filters as filter

def generate_gaussian_2d(kernel_size, sigma):
    #Generate dirac delta function
    dirac_input = np.zeros((kernel_size, kernel_size))
    dirac_input[kernel_size // 2, kernel_size // 2] = 1

    #Convolute dirac delta function with guassian filter will
    #give back the gaussian mask
    gaussian = filter.gaussian_filter(dirac_input, sigma)

    #normalization
    normal = gaussian[kernel_size // 2][kernel_size // 2]
    for entry in gaussian:
        entry /= normal

    return gaussian

#Generate guassian matrix
gaussian = generate_gaussian_2d(16, 3)

#Print gaussian matrix
np.set_printoptions(formatter={'float': '{: 0.5f}'.format})
#Convert to C style syntax
c_matrix = str(gaussian).replace('[', '{')
c_matrix = c_matrix.replace(']', '}')
print(c_matrix)

#Plot
plot.imshow(gaussian, interpolation='none')
plot.show()
