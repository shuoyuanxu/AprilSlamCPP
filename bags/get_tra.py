#!/usr/bin/env python

import rosbag
import csv
import sys
import math
import tf.transformations as tft

def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert a quaternion into a yaw angle in degrees.
    """
    # euler_from_quaternion returns (roll, pitch, yaw) in radians
    roll, pitch, yaw = tft.euler_from_quaternion([qx, qy, qz, qw])
    # Convert yaw (in radians) to degrees if desired:
    yaw_degrees = yaw * 180.0 / math.pi
    return yaw_degrees

def main(bagfile):
    # Change these to match your desired output CSV filenames
    odom_csv_file = "odometry.csv"
    gps_csv_file = "gps.csv"

    # Open the bag
    bag = rosbag.Bag(bagfile, 'r')

    # Prepare CSV writers
    with open(odom_csv_file, 'w', newline='') as f_odom, open(gps_csv_file, 'w', newline='') as f_gps:
        odom_writer = csv.writer(f_odom)
        gps_writer = csv.writer(f_gps)

        # Write headers
        odom_writer.writerow(["time", "x", "y", "z", "heading"])
        gps_writer.writerow(["time", "latitude", "longitude", "altitude"])

        # Read odometry messages
        for topic, msg, t in bag.read_messages(topics=["/antobot_robot/odom"]):
            # nav_msgs/Odometry
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z

            # Extract orientation to compute heading
            ox = msg.pose.pose.orientation.x
            oy = msg.pose.pose.orientation.y
            oz = msg.pose.pose.orientation.z
            ow = msg.pose.pose.orientation.w
            heading = quaternion_to_yaw(ox, oy, oz, ow)

            odom_writer.writerow([t.to_sec(), px, py, pz, heading])

        # Read GPS messages
        for topic, msg, t in bag.read_messages(topics=["/antobot_gps"]):
            # If the message type is sensor_msgs/NavSatFix:
            # msg.latitude, msg.longitude, msg.altitude
            gps_writer.writerow([t.to_sec(), msg.latitude, msg.longitude, msg.altitude])

    bag.close()
    print("Extraction complete!")
    print("Odometry saved to:", odom_csv_file)
    print("GPS saved to:", gps_csv_file)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python extract_to_csv.py <bagfile>")
        sys.exit(1)

    bagfile = sys.argv[1]
    main(bagfile)

