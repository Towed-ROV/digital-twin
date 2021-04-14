# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 13:35:48 2021

@author: sophu
"""
import numpy as np
import matplotlib.pyplot as plt
import random
import cv2


def noise(data, amp):
    """
    Add noise to data.

    Parameters.
    ----------
    data : TYPE
        DESCRIPTION.

    Returns
    -------
    data : TYPE
        DESCRIPTION.

    """
    s1 = amp * np.sin(np.divide(range(len(data)), 100)) * 1.5
    s2 = amp * np.sin(np.divide((range(len(data))), 5)) * 2
    s3 = amp * np.sin(range(len(data))) * 0.05 + 1
    for i, d in enumerate(data):
        data[i] *= s3[i]
        data[i] *= s3[(data.size) - 1 - i]
        data[i] = d + random.choice(np.append([s1[i], s1[i], s2[i]], np.zeros(25)))
    data = data + np.random.normal(0, amp, size=data.size)
    return data


def get_seafloor(max_depth=60, min_depth=30, length=100, samples_per_meter=100, add_noise = True, noise_amp=1, add_edge=True):
    """
    Generate seafloor.

    Parameters.
    ----------
    max_depth : TYPE
        DESCRIPTION.
    min_depth : TYPE
        DESCRIPTION.
    length : TYPE
        DESCRIPTION.
    max_change_per_meter : TYPE
        DESCRIPTION.

    Returns
    -------
    seafloor : TYPE
        DESCRIPTION.
    surface : TYPE
        DESCRIPTION.

    """
    if max_depth < min_depth:
        raise Exception("max depth must be lower than min depth")
    resolution = samples_per_meter*length
    print(resolution)
    resolution = int(resolution*7/10) if add_edge else int(resolution)

    print(length, resolution, samples_per_meter)
    print(resolution)
    median = (max_depth + min_depth) / 2
    t = np.linspace(0, length, resolution)
    seafloor = median * np.ones(resolution)

    seafloor += random.randint(5, 15) * np.sin(
        t / (resolution / random.randint(int(0.3 * samples_per_meter) or 1, int(0.5 * samples_per_meter) or 2)))

    seafloor += random.randint(-3, 3) * np.sin(
        t / (resolution / random.randint(int(4 * samples_per_meter), int(6 * samples_per_meter))))

    seafloor *= 1 + np.abs(
        np.cos(t / (resolution / random.randint(int(8 * samples_per_meter), int(12 * samples_per_meter))))) / 10

    seafloor *= random.random() + 0.3 + np.abs(
        np.cos(t / (resolution / random.randint(int(3 * samples_per_meter), int(5 * samples_per_meter))))) / 5

    seafloor += random.randint(10, 30) * np.sin(
        t // length / (resolution / random.randint(int(45 * samples_per_meter), int(55 * samples_per_meter))))

    mean_v = int(resolution / (samples_per_meter * (50 / (1 + samples_per_meter))))
    mean_v = 1 if not mean_v else mean_v
    seafloor = np.array([np.nanmean(seafloor[i - mean_v:i + mean_v])
                         for i, v in enumerate(seafloor)
                         if i > mean_v + 1 and i < resolution - (mean_v + 1)])

    d = median - np.mean(seafloor)

    seafloor += d
    if add_edge:
        seafloor = np.append(seafloor, np.ones(int(length * samples_per_meter / 10))
                             * seafloor[len(seafloor) - 1])
        seafloor = np.append(seafloor, np.ones(int(length * samples_per_meter / 10))
                             * seafloor[len(seafloor) - 1] + 10)
        seafloor = np.append(seafloor, np.ones(int(length * samples_per_meter / 10))
                             * seafloor[len(seafloor) - 1] - 20)

    mini = np.min(seafloor)

    maxi = np.max(seafloor) - mini

    seafloor = ((seafloor - mini)/maxi) * (max_depth - min_depth) + min_depth
    dif = int(length * samples_per_meter - len(seafloor))

    if dif is not 0:
        seafloor = np.append(seafloor ,np.ones(dif) * seafloor[len(seafloor)-1])
    if add_noise:
        return noise(seafloor.copy())

    return seafloor


def image_builder(filename, seafloor, width,show=False):
    mini = np.min(seafloor)
    maxi = np.max(seafloor) - mini
    seafloor = seafloor - mini
    seafloor = np.divide(seafloor, maxi)
    seafloor = seafloor * 0.9
    img = np.zeros((width, len(seafloor)))
    for i in range(width):
        img[i] = seafloor
    cv2.imwrite(filename, img*255)
    if show:
        cv2.imshow(filename,img)
        cv2.waitKey(0)

def get_and_build(width, length, resolution):
    seafloor = get_seafloor(min_depth=60,max_depth=100,length= length, samples_per_meter=resolution, add_noise= False,
                             add_edge= False)
    image_builder("seafloor.png", seafloor, width)

if __name__ == "__main__":
    seafloor = get_seafloor(length=100, samples_per_meter=10,noise_amp=2)
    print(seafloor)
    #print(min(seafloor), max(seafloor))
    plt.figure()
    if type(seafloor) is list:
        plt.plot(-seafloor[0])
        plt.plot(-seafloor[1])
    else:
        plt.plot(-seafloor)
    plt.show()
    image_builder("../../../AGX-towed-rov-simulation/seafloor.png", seafloor, len(seafloor), show=True)