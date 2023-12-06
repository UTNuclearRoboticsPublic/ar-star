# AR-STAR ROS Package

## Topic Configuration
See [ar-star/config/ar_star_rf_client_config.yaml](https://github.com/UTNuclearRoboticsPublic/ar-star/blob/devel/config/ar_star_rf_client_config.yaml) for a Robotfleet topic config file.

### Robot &rarr; HoloLens
* Send robot repair *detection* on
  ```yaml
  
  /<robot_namespace>/detection  #[augre_msgs/DetectedItem]
  ```
  - ```type``` : "unprocessed" or "masked"
  - ```type label``` : "robot_repair_object"
  - ```how label``` : <robot_namespace>

* Send robot repair plan *point cloud* on
  ```yaml
  /<robot_namespace>/surface_repair/surface_points  #[sensor_msgs/PointCloud2]
  ``` 

* Send robot repair *goal pose* on
  ```yaml
  /<robot_namespace>/goal_pose  #[geometry_msgs/PoseStamped]
  ```
* Send to declare which modification modality to use *[User Study Topic]*
  ```yaml
  /<robot_namespace>/surface_repair/study_modality  #[std_msgs/Header] 
  ```
  - ```seq``` : study type unique id
  - ```frame_id``` : "shape", "highlight", or "lasso"

### HoloLens &rarr; Robot
* Send user feedback to robot
  ```yaml
  /hololens/surface_repair/user_feedback  #[std_msgs/UInt8] 
  ```
  - ```0``` : Accept & Repair
  - ```1``` : Modify
  - ```2``` : Reject

* Send shape information for exclusion or inclusion volumes
  ```yaml
  /hololens/surface_repair/bounding_objects  #[augre_msgs/BoundingObject3DArray] 
  ```

* Send centroids of spheres used for highlighting
  ```yaml
  /hololens/surface_repair/highlight/points  #[geometry_msgs/PolygonStamped] 
  ```

* Send centroids of spheres used for lasso
  ```yaml
  /hololens/surface_repair/lasso/points  #[geometry_msgs/PolygonStamped] 
  ```

* Send an array of points that are highlighted in point cloud
  ```yaml
  /server/surface_repair/highlight/pc_points_tagged  #[std_msgs/UInt8MultiArray] 
  ```
  - ```0``` : points were not highlighted
  - ```1``` : points are in highlight

* Send an array of points that are inside the convex lasso in point cloud
  ```yaml
  /server/surface_repair/lasso/pc_points_tagged  #[std_msgs/UInt8MultiArray] 
  ```
  - ```0``` : points were not inside the convex lasso
  - ```1``` : points are in the convex lasso

* Send upon hearing "end trial" keyword *[User Study Topic]*
  ```yaml
  /hololens/surface_repair/end_trial  #[std_msgs/Header]
  ```
  - ```seq``` : study type unique id
  - ```stamp.sec``` : number of seconds in duration (stop - start time)
  - ```stamp.nsec``` : number of nano seconds in duration (stop - start time)
  - ```frame_id``` : "shape", "highlight", or "lasso"

  
    
