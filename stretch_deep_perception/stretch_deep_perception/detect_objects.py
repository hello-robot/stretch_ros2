#!/usr/bin/env python3

import rclpy

from . import object_detect_pytorch as od
from . import detection_node as dn

def main():
    confidence_threshold = 0.0
    detector = od.ObjectDetector(confidence_threshold=confidence_threshold)
    default_marker_name = 'object'
    node_name = 'DetectObjectsNode'
    topic_base_name = 'objects'
    fit_plane = False
    node = dn.DetectionNode(detector, default_marker_name, node_name, topic_base_name, fit_plane)
    node.main()

    # try:
    #     rclpy.spin(node.node)
    # except KeyboardInterrupt:
    #     print('interrupt received, so shutting down')

if __name__ == '__main__':
    main()
