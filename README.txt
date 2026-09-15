===============================================================================
                         DURIAN INSPECTION SYSTEM
===============================================================================

Autonomous ROS 2 durian-tree inspection using RGB-D capture, deferred YOLO
processing, SQLite result storage, and a Flutter monitoring application.


-------------------------------------------------------------------------------
1. PROJECT OVERVIEW
-------------------------------------------------------------------------------

This project controls a mobile robot that discovers durian trees, navigates to
each tree, captures synchronized RGB and depth observations, and classifies the
inspection results after returning home.

Image capture and model inference are separated into two ROS 2 nodes:

  vision_node.py
      Performs lightweight online RGB-D capture and durable evidence storage.

  processing_node.py
      Performs deferred tree, fruit, and leaf-disease inference using the saved
      observations.

This separation keeps robot scanning responsive while ensuring that every tree
has a clear and recoverable processing state.


-------------------------------------------------------------------------------
2. SYSTEM FLOW
-------------------------------------------------------------------------------

  Tree discovery and navigation
              |
              v
  Inspection Server selects and locks a target tree
              |
              v
  Robot stops at a safe scanning position
              |
              v
  +--------------------------- VISION CAPTURE ----------------------------+
  | Camera RGB + Depth + Camera Info                                      |
  |                  |                                                    |
  |                  v                                                    |
  |       Synchronize RGB-D timestamps                                    |
  |                  |                                                    |
  |                  v                                                    |
  |       Save RGB, Depth, TF and metadata                                |
  |                  |                                                    |
  |                  v                                                    |
  |       captured_images/<inspection_id>/<tree_id>/                      |
  |       captured_observations.status = CAPTURED                         |
  +-----------------------------------------------------------------------+
              |
              v
  inspection_log.status = processing
              |
              v
  Robot inspects the remaining trees and returns home
              |
              v
  +------------------------- VISION PROCESSING ---------------------------+
  | Trigger process_captured_images service                               |
  |                  |                                                    |
  |                  v                                                    |
  |       Validate hashes and restore RGB-D context                       |
  |                  |                                                    |
  |                  v                                                    |
  |       Tree YOLO -> Distance -> Target association -> Tree ROI         |
  |                  |                                                    |
  |                  +--------------------+                               |
  |                  |                    |                               |
  |                  v                    v                               |
  |          Fruit detection      Leaf-disease detection                  |
  |                  |                    |                               |
  |                  +--------------------+                               |
  |                               |                                       |
  |                               v                                       |
  |                    Aggregate per-tree evidence                        |
  |                               |                                       |
  |                               v                                       |
  |       processed_images/<inspection_id>/<tree_id>/                     |
  +-----------------------------------------------------------------------+
              |
              v
  Store disease, coverage, confidence, priority and remedy in SQLite
              |
              v
  inspection_log.status = completed
              |
              v
  Flutter application displays the inspection result


-------------------------------------------------------------------------------
3. TREE STATUS CONTRACT
-------------------------------------------------------------------------------

  processing
      Capture for the tree is complete, but deferred inference has not yet
      produced a durable final result.

  completed
      The final disease result and supporting values have been stored
      successfully and are ready for the Flutter application.

  processing + processing_error
      Processing failed. The error is recorded and the tree deliberately
      remains "processing" instead of being incorrectly marked "completed".

Normal transition:

      processing  ------------------------------>  completed
                     successful durable result


-------------------------------------------------------------------------------
4. MAIN ROS 2 COMPONENTS
-------------------------------------------------------------------------------

  src/durian_inspection_pkg/inspection_server.py
      Discovers trees, selects targets, controls navigation and scanning,
      returns the robot home, and triggers deferred processing.

  src/durian_inspection_pkg/vision_node.py
      Receives RGB, depth and camera information; synchronizes frames; records
      TF and metadata; calculates integrity hashes; and stores observations.
      It does not execute YOLO inference.

  src/durian_inspection_pkg/processing_node.py
      Loads captured observations and executes tree detection, depth-based
      distance calculation, target association, Tree ROI cropping, fruit
      detection, multi-scale leaf-disease detection and result aggregation.

  src/durian_inspection_pkg/observation_store.py
      Creates and migrates the SQLite schema used for observation records,
      inspection results, processing errors and reusable processing cache.

  src/durian_inspection_pkg/bridge_node.py
      Provides communication between the ROS 2 system and user-interface API.

  src/durian_inspection_pkg/map_publisher.py
      Publishes the stored point-cloud map used by the inspection system.

  src/durian_inspection_pkg/collector.py
      Supports controlled RGB-D data collection for Gazebo experiments.

  durian_flutter_app/
      Flutter application for inspection monitoring and result display.


-------------------------------------------------------------------------------
5. REPOSITORY STRUCTURE
-------------------------------------------------------------------------------

  durian_ws/
  |
  +-- README.txt
  +-- run.sh
  +-- models/
  |   +-- durian_leaf/
  |       +-- original.pt
  |       +-- baseline.pt
  |       +-- advanced.pt
  |
  +-- src/
  |   +-- durian_inspection_pkg/
  |   +-- durian_message/
  |
  +-- durian_flutter_app/
  +-- captured_images/          generated at runtime; not stored in Git
  +-- processed_images/         generated at runtime; not stored in Git
  +-- build/                    generated by colcon; not stored in Git
  +-- install/                  generated by colcon; not stored in Git
  +-- log/                      generated by colcon; not stored in Git
  +-- experiment/               local training work; excluded from GitHub
  +-- hybrid_datasets/          local dataset; excluded from GitHub
  +-- external_datasets/        local dataset; excluded from GitHub


-------------------------------------------------------------------------------
6. MODEL CHECKPOINTS
-------------------------------------------------------------------------------

The following six-class durian leaf checkpoints are stored with Git LFS:

  models/durian_leaf/original.pt
      YOLO model trained with the original Gazebo dataset.

  models/durian_leaf/baseline.pt
      YOLO model trained with the balanced real and Gazebo dataset.

  models/durian_leaf/advanced.pt
      Advanced six-class checkpoint intended for the EfficientNet-backed
      experiment.

Other *.pt and *.onnx files remain excluded unless they are explicitly added to
Git LFS.


-------------------------------------------------------------------------------
7. REQUIREMENTS
-------------------------------------------------------------------------------

Recommended environment:

  - Ubuntu 22.04
  - ROS 2 Humble
  - Gazebo Classic with gazebo_ros
  - Python 3
  - OpenCV and cv_bridge
  - NumPy and scikit-learn
  - PyTorch and Ultralytics
  - Navigation2 and TF2
  - Git LFS
  - Flutter SDK for the mobile application

The exact model and ROS dependencies must be available before launching the
complete inspection pipeline.


-------------------------------------------------------------------------------
8. CLONE THE PROJECT
-------------------------------------------------------------------------------

Install and enable Git LFS before cloning or pulling model checkpoints:

  sudo apt-get update
  sudo apt-get install -y git-lfs
  git lfs install

Clone the repository:

  git clone https://github.com/JohnyChia/fyp.git
  cd fyp
  git lfs pull

If the workspace must be located at the path expected by the current launch
configuration, clone or move it to:

  /home/johny/durian_ws


-------------------------------------------------------------------------------
9. BUILD AND RUN
-------------------------------------------------------------------------------

Make the startup script executable once:

  chmod +x run.sh

Start the complete system:

  ./run.sh

The startup script performs these operations:

  1. Stops old robot-related processes.
  2. Clears stale Fast DDS shared-memory files.
  3. Removes previous build, install and log outputs.
  4. Builds the workspace with colcon --symlink-install.
  5. Sources the ROS 2 workspace and Gazebo environment.
  6. Configures DDS and Gazebo environment variables.
  7. Performs a short DDS sanity check.
  8. Launches navigation_launch.py.

Manual build and launch:

  cd /home/johny/durian_ws
  colcon build --symlink-install
  source install/setup.bash
  source /usr/share/gazebo/setup.sh
  ros2 launch durian_inspection_pkg navigation_launch.py


-------------------------------------------------------------------------------
10. RUNTIME DATA
-------------------------------------------------------------------------------

Captured observations:

  captured_images/<inspection_id>/<tree_id>/
      <observation_id>_rgb.png
      <observation_id>_depth.tiff
      <observation_id>.json

Processed observations:

  processed_images/<inspection_id>/<tree_id>/
      <observation_id>_processed.jpg

SQLite database:

  durian_inspection.db

Important SQLite tables:

  captured_observations
      Stores file paths, timestamps, hashes and observation processing state.

  inspection_log
      Stores one result row per inspected tree, including its tree-level status.

  tree_processing_cache
      Reuses a previous result only when both the observation batch and pipeline
      fingerprint match.


-------------------------------------------------------------------------------
11. DATA INTEGRITY AND FAILURE HANDLING
-------------------------------------------------------------------------------

  - RGB, depth and metadata files are written atomically.
  - SHA-256 hashes are verified before deferred processing.
  - Exact duplicate observations are recorded instead of silently reused as
    independent evidence.
  - Cache entries depend on both observation content and pipeline fingerprint.
  - A failed tree remains "processing" and receives a processing_error value.
  - A tree becomes "completed" only after its final result is stored durably.


-------------------------------------------------------------------------------
12. COMMON ISSUES
-------------------------------------------------------------------------------

Permission denied when running ./run.sh

  chmod +x run.sh

Gazebo reports that the shader library is missing

  source /usr/share/gazebo/setup.sh

Model files appear as small text pointers after cloning

  git lfs install
  git lfs pull

VS Code still shows files as modified after a successful push

  git status

If Git reports a clean working tree, save all open editor buffers and refresh
the VS Code Source Control panel or reload the VS Code window.


-------------------------------------------------------------------------------
13. GITHUB REPOSITORY
-------------------------------------------------------------------------------

  https://github.com/JohnyChia/fyp

===============================================================================
