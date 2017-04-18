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

    return 2 - gaussian

def generate_linear_weighting_2d(size):
    #Generate dirac delta function
    linear_weighting = np.zeros((size, size))

    for i in range(-size / 2, size / 2 + 1):
        for j in range(-size / 2, size / 2 + 1):
            linear_weighting[i + size / 2][j + size / 2] = (abs(i) + 1.0) * (abs(j) + 1.0)

    return linear_weighting

#Generate distance weighting functions
linear = generate_linear_weighting_2d(9)
gaussian = generate_gaussian_2d(9, 3)

#Print gaussian matrix
select_print = linear
#select_print = gaussian

#Print in C style syntax
c_matrix = str(select_print).replace('[', '{')
c_matrix = c_matrix.replace(']', '}')
c_matrix = c_matrix.replace('.', ',')
print(c_matrix)

#Plot
plot.figure()
plot.subplot(1, 2, 1)
plot.imshow(linear, interpolation='none')
plot.title("linear distance weighting")

plot.subplot(1, 2, 2)
plot.imshow(gaussian, interpolation='none')
plot.title("gaussian distance weighting")

plot.show()
