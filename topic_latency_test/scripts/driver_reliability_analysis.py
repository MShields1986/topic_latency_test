#!/usr/bin/env python3
"""
Analyse KUKA driver reliability from Test 1 (topic latency) data.

Reliability is inferred from the inter-message interval of the driver's own
'Packet Created' (p1) timestamp.  Whenever a gap between consecutive p1 values
is larger than 1.5 × the nominal publish period, one or more packets were
dropped by the driver or lost in transit before the header timestamp was set.
The total number of expected packets is estimated by dividing each gap by the
nominal period and rounding to the nearest integer.

  reliability = packets_received / packets_expected
"""

import os
import numpy as np
import pandas as pd

DATA_DIR = os.path.join(os.path.dirname(__file__), '..', 'data')

DATASETS = [
    # (label, file, connection)
    ('/odom',                             'odom_wired.txt',               'Wired'),
    ('/odom',                             'odom_wifi.txt',                'Wi-Fi'),
    ('/scan\\_front',                     'scan_front_wired.txt',         'Wired'),
    ('/scan\\_front',                     'scan_front_wifi.txt',          'Wi-Fi'),
    ('/scan\\_rear',                      'scan_rear_wired.txt',          'Wired'),
    ('/scan\\_rear',                      'scan_rear_wifi.txt',           'Wi-Fi'),
    ('/iiwa/state/CartesianPose',         'iiwa_cartesian_pose_wired.txt','Wired'),
    ('/iiwa/state/CartesianPose',         'iiwa_cartesian_pose_wifi.txt', 'Wi-Fi'),
]

print(f"{'Topic':<35} {'Conn':<7} {'N_recv':>7} {'N_exp':>7} {'N_drop':>7} "
      f"{'Rel [%]':>9} {'Nom dt [ms]':>12} {'Max gap [ms]':>13}")
print('-' * 105)

results = []

for label, fname, conn in DATASETS:
    fpath = os.path.join(DATA_DIR, fname)
    df = pd.read_csv(fpath, header=0, sep=', ', engine='python')
    col = 'Packet Created'

    t = df[col].values
    dts = np.diff(t)                          # N-1 inter-packet intervals

    # Nominal period: median of all gaps that are "small"
    # Use iterative refinement: first pass uses full median, second excludes outliers
    nom = np.median(dts)
    # Exclude gaps > 3x median to get a cleaner nominal estimate
    mask_normal = dts < 3.0 * nom
    if mask_normal.sum() > 0:
        nom = np.median(dts[mask_normal])

    # Estimate expected packets per gap: round(gap / nom)
    expected_per_gap = np.round(dts / nom).astype(int)
    expected_per_gap = np.maximum(expected_per_gap, 1)  # each gap is >= 1 packet

    n_recv = len(t)
    n_exp  = int(expected_per_gap.sum()) + 1   # +1 for the first packet
    n_drop = n_exp - n_recv

    reliability = 100.0 * n_recv / n_exp if n_exp > 0 else 0.0

    results.append({
        'label':       label,
        'conn':        conn,
        'n_recv':      n_recv,
        'n_exp':       n_exp,
        'n_drop':      n_drop,
        'reliability': reliability,
        'nom_dt_ms':   nom * 1000,
        'max_gap_ms':  dts.max() * 1000,
    })

    print(f"{label:<35} {conn:<7} {n_recv:>7,} {n_exp:>7,} {n_drop:>7,} "
          f"{reliability:>9.4f} {nom*1000:>12.3f} {dts.max()*1000:>13.3f}")

print()

# Also print clean LaTeX-friendly table values
print("\n=== LaTeX table values ===\n")
print(f"{'Topic':<35} {'Conn':<7} {'N_recv':>8} {'N_exp':>8} {'N_drop':>8} "
      f"{'Rel [%]':>10} {'Nom dt [ms]':>12}")
print('-' * 95)
for r in results:
    print(f"{r['label']:<35} {r['conn']:<7} {r['n_recv']:>8,} {r['n_exp']:>8,} "
          f"{r['n_drop']:>8,} {r['reliability']:>10.3f} {r['nom_dt_ms']:>12.3f}")
