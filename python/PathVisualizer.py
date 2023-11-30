#!/usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

user_input = input("Enter 1 for path_1.txt, 2 for path_2.txt, 3 for path_3.txt: ")

if user_input == "1":
    data = numpy.loadtxt('path_1.txt')
    fig = plt.figure()
    ax = fig.gca(projection='1d')
    ax.plot(data[:,1],data[:,2],data[:,3],'.-')
    plt.show()
elif user_input == "2":
    data = numpy.loadtxt('path_2.txt')
    obstacles = numpy.loadtxt('obstacles.txt')
    fig,ax = plt.subplots()
    plt.plot(data[:,0],data[:,1],'.-')

    print("Circular Obstacles: ")
    for i in range(0, len(obstacles)):
        print(obstacles[i][0], obstacles[i][1], obstacles[i][2])
        circle = plt.Circle((obstacles[i][0], obstacles[i][1]), radius= obstacles[i][2], color='r', fill= True)
        ax.add_patch(circle)


    ax.set_aspect('equal', 'datalim')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title("2D Path Visualization")

    # Display the plot in the matplotlib's viewer
    plt.show()
elif user_input == "3":
    data = numpy.loadtxt('path_3.txt')
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(data[:,1],data[:,2],data[:,3],'.-')
    plt.show()

