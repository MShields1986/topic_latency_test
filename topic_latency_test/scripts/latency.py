#!/usr/bin/env python3

import rospkg
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

###############################################################################
# Plotting Styles
###############################################################################

# Global plot variables
pointsize = 0.8
linewidth = 1
alpha = 0.7
figw = 7 * 1.2 * 1.5
figh = 4 * 1.2
dpi = 400

def histogram(data, xmin, xmax, binsize, xtick):
    name = data.name.split("(")[0].rstrip()

    plt.figure(figsize=(figw, figh))
    #plt.rcParams["axes.facecolor"] = "black"
    plt.grid(True, which="both", alpha=0.3)
    plt.xticks(np.arange(xmin, xmax, step=xtick))

    plt.hist(
        data, bins=np.arange(xmin, xmax, step=binsize), alpha=alpha, color="red"
    )

    plt.suptitle(name + " Histogram")
    plt.title(
        "1σ = "
        + str(round(data.std(), 5))
        + " | mean = "
        + str(round(data.mean(), 5))
        + " | median = "
        + str(round(data.median(), 5))
        + " | n = "
        + str(len(data))
    )
    plt.xlabel(data.name)
    plt.ylabel("Observations")

    plt.savefig(path + file.split(".")[0] + " - " + name + " Histogram.png", dpi=dpi)
    # plt.show()
    plt.close()

def scatter(xdata, ydata):
    xname = xdata.name.split("(")[0].rstrip()
    yname = ydata.name.split("(")[0].rstrip()

    plt.figure(figsize=(figw, figh))
    #plt.rcParams["axes.facecolor"] = "black"
    plt.grid(True, which="both", alpha=0.3)

    plt.scatter(xdata, ydata, c="red", marker=".", alpha=alpha, s=pointsize)

    plt.title(xname + " vs. " + yname + " Scatter Plot")
    plt.xlabel(xdata.name)
    plt.ylabel(ydata.name)
    plt.xlim((xdata.min(), xdata.max()))
    plt.ylim((ydata.min(), ydata.max()))

    plt.savefig(
        path + file.split(".")[0] + " - " + xname + " vs " + yname + " Scatter Plot.png",
        dpi=dpi,
    )
    # plt.show()
    plt.close()

def line(xdata, ydata):
    xname = xdata.name.split("(")[0].rstrip()
    yname = ydata.name.split("(")[0].rstrip()

    plt.figure(figsize=(figw, figh))
    # plt.rcParams["axes.facecolor"] = "black"
    plt.grid(True, which="both", alpha=0.3)

    plt.plot(xdata, ydata, c="red", alpha=alpha, linewidth=linewidth, linestyle="-")
    # plt.plot(xdata, ydata, c="red", alpha=alpha, linewidth=linewidth, linestyle="--")

    plt.title(xname + " vs. " + yname + " Line Plot")
    plt.xlabel(xdata.name)
    plt.ylabel(ydata.name)
    plt.xlim((xdata.min(), xdata.max()))
    plt.ylim((ydata.min(), ydata.max()))

    plt.savefig(
        path + file.split(".")[0] + " - " + xname + " vs " + yname + " Line Plot.png",
        dpi=dpi,
    )
    # plt.show()
    plt.close()

###############################################################################
# Processing
###############################################################################

# Parsing
package_path = rospkg.RosPack().get_path('topic_latency_test')

path = f"{package_path}/data/"

file = "odom.txt"
# file = "scan_front.txt"
# file = "scan_rear.txt"

data = pd.read_csv(f"{path}{file}", header=0, sep=", ", engine='python')

df_length = len(data['Packet Created'])

data['Packet Created dt (sec)'] = data['Packet Created'][1:df_length].reset_index(drop=True) - data['Packet Created'][0:df_length-1]
data['Packet Created dt (sec)'] = data['Packet Created dt (sec)'].shift(1)

data['Packet Received dt (sec)'] = data['Packet Received'][1:df_length].reset_index(drop=True) - data['Packet Received'][0:df_length-1]
data['Packet Received dt (sec)'] = data['Packet Received dt (sec)'].shift(1)

data['Packet Transit dt (sec)'] = data['Packet Created dt (sec)'] - data['Packet Received dt (sec)']

# Plotting
histogram(data['Latency (sec)'], 0, 0.2, 0.001, 0.01)
histogram(data['Packet Created dt (sec)'], 0, 0.1, 0.001, 0.01)
histogram(data['Packet Received dt (sec)'], 0, 0.1, 0.001, 0.01)
histogram(data['Packet Transit dt (sec)'], -0.1, 0.1, 0.001, 0.01)

scatter(data['Packet Created'], data['Latency (sec)'])
scatter(data['Packet Created'], data['Packet Created dt (sec)'])
scatter(data['Packet Created'], data['Packet Received dt (sec)'])
scatter(data['Packet Created'], data['Packet Transit dt (sec)'])

line(data['Packet Created'], data['Packet Transit dt (sec)'])

scatter(data['Latency (sec)'], data['Packet Created dt (sec)'])
scatter(data['Latency (sec)'], data['Packet Received dt (sec)'])
scatter(data['Packet Created dt (sec)'], data['Packet Received dt (sec)'])
