#!/usr/bin/env python

import rosbag

def remove_tf_messages(input_bag, output_bag):
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == '/ms/imu/data':
                # Skip /tf messages
                continue
            # Otherwise, keep the message
            outbag.write(topic, msg, t)

if __name__ == "__main__":
    input_bag_path = "medium_DLO.bag"
    output_bag_path = "medium_DLO_1.bag"

    remove_tf_messages(input_bag_path, output_bag_path)
    print(f"Created {output_bag_path} without the /ms/imu/data topic.")

