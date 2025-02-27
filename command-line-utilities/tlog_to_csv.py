# Author: Louis Lapp
# Description: Convert MAVLink tlog to csv

from pymavlink import mavutil
import pandas as pd
import argparse

def tlog_to_csv(tlog_file, output_csv):
    mav = mavutil.mavlink_connection(tlog_file)

    log_dict = {}
    timestamps = []

    while msg := mav.recv_match():
        timestamp = getattr(msg, '_timestamp', None)
        timestamps.append(timestamp)

        msg_dict = msg.to_dict()

        for key, value in msg_dict.items():
            if key not in log_dict:
                log_dict[key] = []
            log_dict[key].append(value)

    max_length = len(timestamps)
    for key in log_dict.keys():
        while len(log_dict[key]) < max_length:
            log_dict[key].append(None)

    df = pd.DataFrame(log_dict, index=pd.to_datetime(timestamps, origin="unix", unit="s"))
    df.to_csv(output_csv)

parser = argparse.ArgumentParser(description= "Convert MAVLink tlog to csv")

parser.add_argument('tlog_file', type=str, help="File location of tlog")
parser.add_argument('output_csv', type=str, help="Save directory for csv")

args = parser.parse_args()

df = tlog_to_csv(args.tlog_file, args.output_csv)