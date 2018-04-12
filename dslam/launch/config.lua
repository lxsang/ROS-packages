local PI = 3.1415926535898
local config = {
    scan_topic = "/scan",
    keypoint_topic = "keypoints",
    odom_topic = "/odom" , --  is used for  correction before scan matching
    localisation = {
        line_extraction = {
            bearing_std_dev = 1e-3,
            range_std_dev = 0.02,
            least_sq_angle_thresh = 1e-4,
            least_sq_radius_thresh = 1e-4,
            max_line_gap = 0.4,
            min_line_length = 0.5,
            min_range = 0.4,
            min_split_dist = 0.05,
            outlier_dist = 0.06,
            min_line_points = 9.0,
            line_scale = 0.05
        },
        point_cloud_extraction = {
            max_range = 8.0 --m
        },
        icp = {
            max_distance = 0.05, -- 5cm correspondence distance
            max_iteration = 250, -- maximum number of iterations 
            tf_epsilon = 1e-6, -- transformation epsilon
            eu_fitness = 0.5, -- the euclidean distance difference epsilon
            sample_fitness = 0.5--, -- 30 cm
            --translation_tolerance = 1e-6
        },
        keyframe_sample_linear = 0.5,
        keyframe_sample_angular = 0.7,
        global_frame = "map",
        laser_frame = "laser",
        robot_base = "base_footprint",
        odom_frame = "odom"
    },
    mapping = {
        local_map = {
            p_occupied_with_observation=0.9,
            p_occupied_without_observation=0.3,
            angle_resolution= PI/720.0,
            large_log_odds=100.0,
            max_log_odds_for_belief=20.0,
            max_laser_range = 5.0, -- meter
            resolution = 0.05
        }
    },
    line_seg_topic = "/line_markers",
    frequency = 15
}
return config