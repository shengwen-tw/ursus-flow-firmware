import numpy as np
import matplotlib.pyplot as plot
import scipy.ndimage.filters as filter

def generate_gaussian_2d(kernel_size, sigma, max_weighting):
    #Generate dirac delta function
    dirac_input = np.zeros((kernel_size, kernel_size))
    dirac_input[kernel_size // 2, kernel_size // 2] = 1

    #Convolute dirac delta function with guassian filter will
    #give back the gaussian mask
    gaussian = filter.gaussian_filter(dirac_input, sigma)

    #normalization
    normal = gaussian[kernel_size // 2][kernel_size // 2]
    for entry in gaussian:
        entry /= normal / max_weighting

    return max_weighting - np.rint(gaussian) + 1

def generate_linear_weighting_2d(size):
    #Generate dirac delta function
    linear_weighting = np.zeros((size, size))

    coefficient = 1

    for i in range(-size / 2, size / 2 + 1):
        for j in range(-size / 2, size / 2 + 1):
            linear_weighting[i + size / 2][j + size / 2] = (coefficient * abs(i) + 1.0) * (coefficient * abs(j) + 1.0)

    return linear_weighting

#Generate distance weighting functions
linear = generate_linear_weighting_2d(9)
gaussian = generate_gaussian_2d(9, 3.4, 35)

#Print matrix
select_print = gaussian
#select_print = linear

#Print in C style syntax
np.set_printoptions(formatter={'float': '{: .1f}'.format})
c_matrix = str(select_print).replace('[', '{')
c_matrix = c_matrix.replace(']', '}')
c_matrix = c_matrix.replace('.0', ',')
c_matrix = c_matrix.replace('}}', ')')
c_matrix = c_matrix.replace('}', '},')
c_matrix = c_matrix.replace(')', '}}')
c_matrix = c_matrix.replace(',}', '}')
c_matrix += ';'

if select_print.all() == gaussian.all():
    print('\n//gaussian distance weighting:\n'
          'const uint8_t distance_weighting_table[9][9] =\n' + c_matrix)

elif select_print.all() == linear.all():
    print('\n//linear distance weighting:\n'
          'const uint8_t distance_weighting_table[9][9] =\n' + c_matrix)

#Plot
plot.figure()
plot.subplot(1, 2, 1)
plot.imshow(linear, interpolation='none')
plot.title("linear distance weighting")

plot.subplot(1, 2, 2)
plot.imshow(gaussian, interpolation='none')
plot.title("gaussian distance weighting")

plot.show()
