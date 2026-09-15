DURIAN INSPECTION SYSTEM
========================

This repository contains a ROS 2 and Gazebo-based autonomous durian tree
inspection system. The robot discovers trees, navigates to each target,
captures synchronized RGB-D observations, returns home, processes the saved
observations with computer-vision models, stores the results in SQLite, and
makes the inspection results available to the Flutter application.


SYSTEM WORKFLOW
===============

1. The Inspection Server discovers a tree and navigates the robot to a safe
   scanning position.
2. vision_node.py receives RGB, depth, camera calibration, inspection ID,
   target tree ID, and scan state information.
3. Synchronized RGB-D observations and their metadata are stored under
   captured_images/<inspection_id>/<tree_id>/.
4. The tree is recorded in SQLite with the tree-level status "processing".
5. After all trees have been scanned, the robot returns home and calls the
   process_captured_images service.
6. processing_node.py loads the captured observations and executes tree,
   fruit, and leaf-disease inference without changing the original detection
   and aggregation logic.
7. Annotated output images are stored under
   processed_images/<inspection_id>/<tree_id>/.
8. When the aggregated result has been stored successfully, the tree-level
   status changes from "processing" to "completed". If processing fails, the
   tree remains "processing" and the error is recorded.


MAIN COMPONENTS
===============

src/durian_inspection_pkg/vision_node.py
    Captures synchronized RGB and depth frames and stores durable observation
    evidence. This node does not perform YOLO inference.

src/durian_inspection_pkg/processing_node.py
    Performs deferred tree detection, distance calculation, Tree ROI cropping,
    fruit detection, multi-scale leaf-disease detection, evidence aggregation,
    processed-image storage, and SQLite result finalization.

src/durian_inspection_pkg/observation_store.py
    Defines and migrates the SQLite tables used for captured observations,
    inspection results, processing errors, and processing cache entries.

src/durian_inspection_pkg/inspection_server.py
    Controls tree discovery, target selection, navigation, scanning, return to
    the home position, and deferred-processing service invocation.

src/durian_inspection_pkg/bridge_node.py
    Provides the bridge used by the user interface to communicate with ROS 2.

src/durian_inspection_pkg/map_publisher.py
    Publishes the stored map point cloud for the inspection system.

durian_flutter_app/
    Contains the Flutter application used to display inspection information and
    communicate with the ROS-facing API.


BUILD AND RUN
=============

The workspace is designed for ROS 2 on Ubuntu with Gazebo Classic available.
From the workspace root, make the startup script executable once:

    chmod +x run.sh

Start the complete system with:

    ./run.sh

The script removes old build outputs, builds the ROS 2 workspace with symlink
installation, sources the workspace and Gazebo environments, performs a DDS
sanity check, and launches navigation_launch.py.

To build manually:

    colcon build --symlink-install
    source install/setup.bash
    source /usr/share/gazebo/setup.sh
    ros2 launch durian_inspection_pkg navigation_launch.py


RUNTIME OUTPUTS
===============

captured_images/
    Raw RGB, depth, and metadata evidence created by vision_node.py.

processed_images/
    Annotated observations created by processing_node.py.

durian_inspection.db
    SQLite database containing captured-observation and inspection-result data.

These runtime outputs are intentionally excluded from Git.


MODELS AND LOCAL DATA
=====================

Trained model files such as *.pt and *.onnx are excluded from this repository.
They must be placed at the paths configured in the ROS launch file or supplied
through ROS parameters before inference is started.

The following local training and dataset directories are also intentionally
excluded from GitHub:

    experiment/
    hybrid_datasets/
    external_datasets/

ROS build outputs, Python caches, Flutter build outputs, databases, logs, point
cloud files, captured images, and processed images are excluded as well.


TREE PROCESSING STATUS
======================

processing
    Image capture for the tree is complete, but deferred model processing has
    not yet produced a durable final result.

completed
    The final disease, coverage, confidence, priority, and remedy values have
    been stored successfully and can be displayed by the Flutter application.

