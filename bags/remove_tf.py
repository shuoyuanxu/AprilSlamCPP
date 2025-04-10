#!/usr/bin/env python

import rosbag

def remove_tf_messages(input_bag, output_bag):
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == '/tf':
                # Skip /tf messages
                continue
            # Otherwise, keep the message
            outbag.write(topic, msg, t)

if __name__ == "__main__":
    input_bag_path = "loc2_dlo_failed_badly.bag"
    output_bag_path = "loc2_dlo_failed_badly_filtered.bag"

    remove_tf_messages(input_bag_path, output_bag_path)
    print(f"Created {output_bag_path} without the /tf topic.")

