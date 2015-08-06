# offline_relational_learner
====
A package for analysing the (G4S) stored trajectory data in an offline manner. 

Prerequisites
-------------
- MongoDB (>=2.4)
- mongodb_store
- ROS's navigation stack (only map server)
- strands_perception_people (mainly need the message definitions)
- soma (soma and soma roi - if you want to visualise objects)
- qsrlib (if you want to generate new QSRs)
- strands_data_to_qsrlib (package used as a parser to qsrlib)
  Will need a `config.ini' options file [here](https://github.com/strands-project/strands_data_to_qsrlib/tree/master/src/novelTrajectories)
- relational_learner (if you want to upload episodes)



Getting started (general steps)
-------------------------------
1. Launch the ROS datacentre:

    ```
    $ roslaunch mongodb_store mongodb_store.launch db_path:=/home/strands/mongodb_store/g4s_mongo_store/
    ```
2. Run the map_server with a 2D map:

  ```
  $ rosrun map_server map_server <map.yaml>
  ```
where `map.yaml` specifies the map you want to load i.e. `/home/strands/STRANDS/deployment_data/maps/g4s/edited`
    
  May also need a static transform from `map` to `frame`:

  ```
  $ rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map frame 100
  ```
 
3. Run the SOMA object (and roi) manager:

    ```
    $ rosrun soma_manager soma.py <map> <config>
    $ rosrun soma_manager soma_roi.py <map> <config>
    ```
where `map` denotes the name of the map and `config` denotes an object configuration within this map. 

----------------


offline_episodes.py
-----------------------------

Can be ran using: 

    ```
    $ rosrun offline_relational_learner offline_episodes.py
    ```

  Code will loop over the people_trajectory data in mongodb, querying a specified range of dates (which need to be edited in `__main__` at the moment).
  
  Trajectories are filtered using `trajectory_displacement` and `displacement_pose_ratio` in the [Trajectory msg](https://github.com/strands-project/strands_perception_people/blob/indigo-devel/human_trajectory/msg/Trajectory.msg).
  
  Episodes are generated and uploaded to mongodb using:
  
    ```
    $ rosrun qsr_lib qsrlib_ros_server.py 
    $ rosrun relational_learner episodes_server.py
    ```
  where qsr and episode options are specified in the `config.ini` file in `strands_data_to_qsrlib`.

----------------


split_trajectory_test_set.py
-----------------------------

Can be ran using: 

    ```
    $ rosrun offline_relational_learner split_trajectory_test_set.py
    ```

  Code requires a pickle file (list) of Trajectory UUIDs to be split by their [Trajectory msg](https://github.com/strands-project/strands_perception_people/blob/indigo-devel/human_trajectory/msg/Trajectory.msg) `sequence_id`.

  Will split Trajectories into mini-batches (using sequence_id), and create the QSRs, and upload them to mongodb with `_test_seq_%s` as part of their UUID.
  
  Similar to above, requires:
    
    ```
    $ rosrun qsr_lib qsrlib_ros_server.py 
    $ rosrun relational_learner episodes_server.py
    ```
  and a `config.ini` file in `strands_data_to_qsrlib`.
 
----------------
  
  
learn_traj_behaviours.py
-----------------------------

Can be ran using: 

    ```
    $ rosrun offline_relational_learner learn_traj_behaviours.py -p 0/1 -e 'episode_store' -l 'kmeans' -q 'qsr_type'
    ```

  where `-p` = plotting. `-e` is a mongodb database to upload episodes. `-l` is the learning method and `-q` is the qsr_type used to query the database.

  Code will query the `episode_store` for episodes with qsr = `qsr_type`. It will generate Activity Graphs for the episodes, and then perform kmeans. 

  It will output a pickle file of the [Learning class](https://github.com/strands-project/trajectory_behaviours/blob/master/relational_learner/src/relational_learner/learningArea.py#L46) 
  in which, `.methods` there will be an sklearn kmeans estimator.
  
This will eventually run both the `visualise_qualitative_predictions.py` and the `offline_kmeans_testing.py` functions. 


----------------
  
  

    ```
    $ rosrun soma_trajectory trajectory_query_service.py
    ```
  
offline_kmeans_testing.py
-----------------------------

Can be ran using: 

    ```
    $ rosrun offline_relational_learner offline_kmeans_testing.py -t TestSet -p 0/1 -v 
    ```
    
  Also requires the trajectory_query_service:
  
    ```
    $ rosrun soma_trajectory trajectory_query_service.py
    ```

  Requires a [Learning class]() object containing an sklearn kmeans estimator. 
  
  Generates an [occupancy_grid object](https://github.com/PDuckworth/trajectory_behaviours/blob/test/offline_relational_learner/scripts/visualise_qualitative_predictions.py#L58) 
  for each cluster, which uses a [QSR mask](https://github.com/PDuckworth/trajectory_behaviours/blob/test/offline_relational_learner/scripts/visualise_qualitative_predictions.py#L92) 
  to visualise the cluster centres as a probability density over the map.
  
  To view in RViz, add a `map` topic and subscribe to `/trajectory_behaviours/occu`.
  
  


