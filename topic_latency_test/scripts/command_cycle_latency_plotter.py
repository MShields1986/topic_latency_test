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

# Data sub-directories and files (new format: Packet Sent, Robot Stamp, Callback Time,
# Latency Robot (sec), Latency Callback (sec))
datasets = [
    ("kmr_command_260515/kmr_command_wired.txt",  "kmr_command_wired"),
    ("kmr_command_260515/kmr_command_wifi.txt",   "kmr_command_wifi"),
    ("iiwa_command_260515/iiwa_command_wired.txt", "iiwa_command_wired"),
    ("iiwa_command_260515/iiwa_command_wifi.txt",  "iiwa_command_wifi"),
]


def cmd_histogram(data_ms, label, xmin, xmax, binsize, xtick):
    plt.figure(figsize=(figw, figh))
    plt.grid(True, which="both", alpha=0.3)
    plt.xticks(np.arange(xmin, xmax + xtick, step=xtick))

    plt.hist(data_ms, bins=np.arange(xmin, xmax + binsize, step=binsize),
             alpha=alpha, color="red")

    plt.suptitle(f"{label} - Command-Observe Latency Histogram")
    plt.title(
        f"1σ = {data_ms.std():.2f} ms"
        f" | mean = {data_ms.mean():.2f} ms"
        f" | median = {data_ms.median():.2f} ms"
        f" | n = {len(data_ms)}"
    )
    plt.xlabel("Command-Observe Loop Latency (ms)")
    plt.ylabel("Observations")
    plt.xlim((xmin, xmax))

    plt.savefig(path + label + " - Latency_Histogram.png", dpi=dpi, bbox_inches='tight')
    plt.close()


for filepath, label in datasets:
    data = pd.read_csv(f"{path}{filepath}", header=0, sep=", ", engine='python')

    # Use Latency Callback (wall-clock end-to-end) converted to ms
    lat_ms = data['Latency Callback (sec)'] * 1000

    # KMR distribution is ~5x tighter than iiwa; 10ms bins avoid the comb pattern
    # that appears at 5ms resolution due to callback timing quantisation
    binsize = 10 if 'kmr' in label else 5
    cmd_histogram(lat_ms, label, xmin=0, xmax=650, binsize=binsize, xtick=50)
