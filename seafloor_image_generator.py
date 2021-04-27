import numpy as np
import cv2


class seafloorImageBuilder:
    def normalize(self, array, new_min=0, new_max=1):
        """
        Normalizes an array, to the new provided min and max.

        Args:
            array: an array of numbers.
            new_min: a number for the new minimum of the array.
            new_max: a number for the new maximum of the array.
        """
        old_min = np.min(array)
        old_max = np.max(array)
        ret_array = np.divide(array - old_min, old_max - old_min)
        ret_array = np.array((ret_array * (new_max - new_min) + new_min))
        return ret_array

    def generate_image(self, array, width=10):
        """
        generate a multimidmetional array row of an array.
        Args:
            array: original array
            width: the new dimention count of the new array.
        """
        img = np.zeros((width, len(array)))
        array = np.array(self.normalize(array, new_min=0, new_max=255), np.uint8)

        for i in range(width):
            img[i] = array
        return img

    def save_new_seafloor_image(self, width, seafloor_array=None):
        """
        generates and saves an image of the seafloor to the project.
        Args:
            width: the width of the image.
            seafloor_array: the seafloor plot to be made to an image.
        """
        if not seafloor_array:
            seafloor_array = self.get_base_seafloor()
        img = self.generate_image(seafloor_array, width)
        cv2.imwrite('seafloor.png', img)

    def get_base_seafloor(self):
        return np.array(
            [14.23, 14.25, 14.27, 14.28, 14.3, 14.31, 14.32, 14.33, 14.34, 14.35, 14.35, 14.36, 14.36, 14.36,
             14.36, 14.36, 14.36, 14.36, 14.35, 14.35, 14.34, 14.33, 14.32, 14.31, 14.3, 14.29, 14.27, 14.26,
             14.25, 14.24, 14.23, 14.22, 14.21, 14.21, 14.2, 14.2, 14.2, 14.2, 14.21, 14.21, 14.22, 14.23, 14.24,
             14.25, 14.27, 14.28, 14.3, 14.32, 14.35, 14.37, 14.39, 14.42, 14.45, 14.48, 14.51, 14.55, 14.59,
             14.62, 14.66, 14.71, 14.75, 14.79, 14.84, 14.89, 14.94, 14.99, 15.05, 15.1, 15.16, 15.22, 15.27,
             15.33, 15.38, 15.44, 15.49, 15.54, 15.59, 15.64, 15.68, 15.73, 15.77, 15.82, 15.86, 15.9, 15.94,
             15.97, 16.01, 16.04, 16.07, 16.11, 16.13, 16.16, 16.19, 16.21, 16.23, 16.26, 16.27, 16.29, 16.31,
             16.32, 16.33, 16.35, 16.35, 16.36, 16.37, 16.37, 16.37, 16.37, 16.37, 16.37, 16.36, 16.35, 16.34,
             16.33, 16.32, 16.31, 16.29, 16.27, 16.25, 16.23, 16.21, 16.18, 16.16, 16.13, 16.1, 16.07, 16.03,
             16.0, 15.96, 15.92, 15.88, 15.84, 15.8, 15.75, 15.7, 15.66, 15.61, 15.56, 15.5, 15.45, 15.39, 15.33,
             15.28, 15.22, 15.15, 15.09, 15.03, 14.96, 14.9, 14.83, 14.76, 14.69, 14.62, 14.54, 14.47, 14.39,
             14.32, 14.24, 14.16, 14.08, 14.01, 13.92, 13.84, 13.76, 13.68, 13.59, 13.51, 13.42, 13.34, 13.26,
             13.18, 13.1, 13.03, 12.95, 12.88, 12.81, 12.74, 12.68, 12.62, 12.56, 12.5, 12.45, 12.4, 12.35,
             12.31, 12.27, 12.23, 12.2, 12.17, 12.14, 12.11, 12.09, 12.07, 12.06, 12.05, 12.04, 12.03, 12.03,
             12.03, 12.03, 12.04, 12.05, 12.06, 12.08, 12.1, 12.12, 12.15, 12.18, 12.21, 12.24, 12.27, 12.31,
             12.34, 12.38, 12.41, 12.45, 12.48, 12.52, 12.56, 12.59, 12.63, 12.66, 12.7, 12.73, 12.76, 12.8,
             12.83, 12.86, 12.88, 12.91, 12.94, 12.96, 12.99, 13.01, 13.03, 13.05, 13.07, 13.09, 13.11, 13.13,
             13.14, 13.16, 13.17, 13.19, 13.2, 13.21, 13.22, 13.23, 13.23, 13.24, 13.24, 13.25, 13.25, 13.25,
             13.25, 13.25, 13.25, 13.25, 13.24, 13.24, 13.23, 13.23, 13.22, 13.21, 13.2, 13.19, 13.17, 13.16,
             13.15, 13.13, 13.12, 13.1, 13.08, 13.06, 13.04, 13.02, 13.0, 12.98, 12.95, 12.93, 12.9, 12.88,
             12.85, 12.82, 12.79, 12.76, 12.73, 12.7, 12.67, 12.64, 12.61, 12.57, 12.54, 12.51, 12.47, 12.43,
             12.4, 12.36, 12.32, 12.29, 12.25, 12.21, 12.17, 12.13, 12.09, 12.05, 12.01, 11.97, 11.93, 11.9,
             11.87, 11.83, 11.81, 11.78, 11.76, 11.73, 11.71, 11.7, 11.68, 11.67, 11.66, 11.65, 11.65, 11.64,
             11.64, 11.64, 11.65, 11.65, 11.66, 11.67, 11.69, 11.7, 11.72, 11.74, 11.76, 11.79, 11.82, 11.85,
             11.88, 11.92, 11.95, 11.99, 12.03, 12.08, 12.12, 12.17, 12.22, 12.28, 12.33, 12.39, 12.45, 12.5,
             12.56, 12.61, 12.67, 12.72, 12.78, 12.83, 12.89, 12.94, 12.99, 13.05, 13.1, 13.15, 13.2, 13.26,
             13.31, 13.36, 13.41, 13.46, 13.5, 13.55, 13.6, 13.65, 13.69, 13.74, 13.78, 13.83, 13.87, 13.92,
             13.96, 14.0, 14.04, 14.08, 14.12, 14.16, 14.2, 14.24, 14.27, 14.31, 14.34, 14.38, 14.41, 14.44,
             14.48, 14.51, 14.54, 14.56, 14.59, 14.62, 14.65, 14.67, 14.7, 14.72, 14.74, 14.77, 14.79, 14.81,
             14.83, 14.84, 14.86, 14.88, 14.89, 14.91, 14.92, 14.93, 14.95, 14.96, 14.97, 14.98, 14.98, 14.99,
             15.0, 15.0, 15.01, 15.01, 15.01, 15.02, 15.02, 15.02, 15.02, 15.02, 15.01, 15.01, 15.01, 15.0, 15.0,
             14.99, 14.98, 14.97, 14.97, 14.96, 14.95, 14.94, 14.93, 14.91, 14.9, 14.89, 14.88, 14.88, 14.87,
             14.87, 14.87, 14.87, 14.88, 14.88, 14.89, 14.9, 14.92, 14.93, 14.95, 14.97, 15.0, 15.02, 15.05,
             15.08, 15.11, 15.15, 15.19, 15.23, 15.27, 15.31, 15.36, 15.41, 15.46, 15.51, 15.57, 15.63, 15.69,
             15.75, 15.82, 15.88, 15.95, 16.03, 16.1, 16.18, 16.25, 16.33, 16.42, 16.5, 16.58, 16.66, 16.74,
             16.82, 16.9, 16.98, 17.05, 17.13, 17.2, 17.28, 17.35, 17.42, 17.49, 17.56, 17.63, 17.69, 17.76,
             17.82, 17.89, 17.95, 18.01, 18.07, 18.13, 18.18, 18.24, 18.29, 18.34, 18.39, 18.44, 18.49, 18.54,
             18.58, 18.62, 18.66, 18.7, 18.74, 18.78, 18.81, 18.84, 18.88, 18.9, 18.93, 18.96, 18.98, 19.0,
             19.02, 19.04, 19.06, 19.08, 19.09, 19.1, 19.11, 19.12, 19.12, 19.13, 19.13, 19.13, 19.13, 19.13,
             19.12, 19.12, 19.11, 19.1, 19.09, 19.07, 19.06, 19.04, 19.02, 19.0, 18.98, 18.95, 18.93, 18.9,
             18.87, 18.84, 18.81, 18.78, 18.74, 18.7, 18.66, 18.62, 18.58, 18.54, 18.5, 18.45, 18.4, 18.35, 18.3,
             18.25, 18.2, 18.15, 18.09, 18.03, 17.98, 17.92, 17.86, 17.8, 17.74, 17.68, 17.63, 17.57, 17.52,
             17.47, 17.43, 17.38, 17.34, 17.3, 17.26, 17.22, 17.19, 17.15, 17.12, 17.09, 17.07, 17.04, 17.02,
             17.0, 16.98, 16.97, 16.95, 16.94, 16.93, 16.93, 16.92, 16.92, 16.92, 16.92, 16.92, 16.93, 16.94,
             16.96, 16.98, 17.0, 17.03, 17.06, 17.09, 17.12, 17.16, 17.21, 17.25, 17.29, 17.34, 17.38, 17.43,
             17.47, 17.52, 17.56, 17.61, 17.65, 17.7, 17.74, 17.79, 17.84, 17.88, 17.93, 17.97, 18.02, 18.07,
             18.11, 18.16, 18.21, 18.25, 18.3, 18.35, 18.39])


if __name__ == "__main__":
    seafloorImageBuilder().save_new_seafloor_image(width=1000)
    img = cv2.imread('seafloor.png', cv2.IMREAD_GRAYSCALE)
    print(img)
