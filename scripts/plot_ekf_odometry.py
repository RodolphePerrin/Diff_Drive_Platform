#!/usr/bin/env python

from matplotlib import pyplot as pp


def load_odometry(path):
    odometry = []

    lines = []
    for line in open(path):
        reading = line.split()
        odometry.append(reading)

    return odometry


def plot_odometry(path_encoders, path_ground_truth, path_filtered):
    
    encoders = load_odometry(path_encoders)
    (t, x_encoders, y_encoders, yaw_encoders) = zip(*encoders)
    
    ground_truth = load_odometry(path_ground_truth)
    (t, x_truth, y_truth, yaw_truth) = zip(*ground_truth)
    filtered = load_odometry(path_filtered)
    print(filtered)
    (t, x_filtered, y_filtered, yaw_filtered) = zip(*filtered)

    pp.plot(x_encoders, y_encoders, 'b.-')
    pp.plot(x_filtered, y_filtered, 'r,-')
    pp.plot(x_truth, y_truth, 'g.-')
    pp.show()

    pp.plot(yaw_encoders, 'b.-')
    pp.plot(yaw_truth, 'r,-')
    pp.plot(yaw_filtered, 'g.-')
    pp.show()


def main():
    from sys import argv
    plot_odometry(*argv[1:])


if __name__ == '__main__':
    main()

