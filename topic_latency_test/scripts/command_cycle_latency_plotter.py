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
figw = 6 #7 * 1.2 * 1.5
figh = 4 #4 * 1.2
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

    plt.suptitle(f"{file.split('.')[0]} - {name} Histogram")
    plt.title(
        "1σ = "
        + str(round(data.std(), 4))
        + " | mean = "
        + str(round(data.mean(), 4))
        + " | median = "
        + str(round(data.median(), 4))
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

    plt.title(f"{file.split('.')[0]} - {xname} vs. {yname} Scatter Plot")
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

    plt.title(f"{file.split('.')[0]} - {xname} vs. {yname} Line Plot")
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

files_wired = ["kmr_command_wired.txt",
               "iiwa_command_wired.txt"]

files_wifi = ["kmr_command_wifi.txt",
              "iiwa_command_wifi.txt"]


for file in files_wifi:
    data = pd.read_csv(f"{path}{file}", header=0, sep=", ", engine='python')
    
    df_length = len(data['Packet Sent'])
    
    data['Latency (ms)'] = data['Latency (sec)'] * 1000
    
    data['Packet Sent dt (sec)'] = data['Packet Sent'][1:df_length].reset_index(drop=True) - data['Packet Sent'][0:df_length-1]
    data['Packet Sent dt (sec)'] = data['Packet Sent dt (sec)'].shift(1)
    
    data['Action Observed dt (sec)'] = data['Action Observed'][1:df_length].reset_index(drop=True) - data['Action Observed'][0:df_length-1]
    data['Action Observed dt (sec)'] = data['Action Observed dt (sec)'].shift(1)
    
    data['Transit dt (sec)'] = data['Packet Sent dt (sec)'] - data['Action Observed dt (sec)']
    data['Transit dt (ms)'] = data['Transit dt (sec)'] * 1000

    # Plotting
    if 'iiwa' in file:
        histogram(data['Latency (ms)'],
                  data['Latency (ms)'].min().round(2),
                  data['Latency (ms)'].max().round(2),
                  0.001,
                  0.1)
        histogram(data['Transit dt (ms)'],
                  data['Transit dt (ms)'].min().round(1),
                  data['Transit dt (ms)'].max().round(1),
                  0.005,
                  0.1)
    else:
        histogram(data['Latency (ms)'],
                  data['Latency (ms)'].min().round(2),
                  data['Latency (ms)'].max().round(2),
                  0.001,
                  0.01)
        histogram(data['Transit dt (ms)'],
                  data['Transit dt (ms)'].min().round(1),
                  data['Transit dt (ms)'].max().round(1),
                  0.001,
                  0.05)

    scatter(data['Packet Sent'], data['Latency (ms)'])
    scatter(data['Packet Sent'], data['Transit dt (ms)'])
