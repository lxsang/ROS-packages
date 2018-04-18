local PI = 3.1415926535898
local config = {
    scan_topic = "/scan",
    keypoint_topic = "keypoints",
    odom_topic = "/odom" , --  is used for  correction before scan matching
    localisation = {
        line_extraction = {
            bearing_std_dev = 1e-3,
            range_std_dev = 0.01, -- 0.1
            least_sq_angle_thresh = 1e-4,
            least_sq_radius_thresh = 1e-4,
            max_line_gap = 0.1, -- 0.4
            min_line_length = 0.3,
            min_range = 0.4,
            min_split_dist = 0.05,
            outlier_dist = 0.03, -- 0.06
            min_line_points = 15.0,
            line_scale = 0.06,
            max_dist = 6.0 -- max laser range
        },
        icp = {
            max_distance = 0.02, -- m correspondence distance
            max_iteration = 250, -- maximum number of iterations 
            tf_epsilon = 1e-6, -- transformation epsilon
            eu_fitness = 1.0, -- the euclidean distance difference epsilon
            sample_fitness = 0.1,-- m
            use_non_linear = true
        },
        pf = {
            MU_SYSTEM_NOISE_X = 0.0,
            MU_SYSTEM_NOISE_Y = 0.0,
            MU_SYSTEM_NOISE_THETA = 0.0,
            SIGMA_SYSTEM_NOISE_X = 4e-6,
            SIGMA_SYSTEM_NOISE_Y = 4e-6,
            SIGMA_SYSTEM_NOISE_THETA = 4e-6 ,-- 4*PI*PI/(180*180),
            MU_MEAS_NOISE_X = 0.0,
            MU_MEAS_NOISE_Y = 0.0,
            MU_MEAS_NOISE_THETA = 0.0,
            SIGMA_MEAS_NOISE_X = 2e-6,
            SIGMA_MEAS_NOISE_Y = 2e-6,
            SIGMA_MEAS_NOISE_THETA = 2e-6,-- 4*PI*PI/(120*120),
            PRIOR_MU_X = -0.01,
            PRIOR_MU_Y = 0.01,
            PRIOR_MU_THETA = PI/4,
            PRIOR_COV_X = 1e-4,
            PRIOR_COV_Y = 1e-4,
            PRIOR_COV_THETA = (PI/8)*(PI/8),
            NUM_SAMPLES = 150
        },
        keyframe_sample_linear = 0.5,
        keyframe_sample_angular = 0.5,
        global_frame = "map",
        laser_frame = "laser",
        robot_base = "base_footprint",
        odom_frame = "odom",
        publish_odom = false
    },
    mapping = {
        p_occupied_with_observation=0.9,
        p_occupied_without_observation=0.2,
        angle_resolution= PI/720.0,
        large_log_odds=90.0,
        max_log_odds_for_belief=60.0,
        max_laser_range = 3.0, -- meter
        resolution = 0.05,
        map_frame = "map",
        init_width = 20.0, --m
        init_height = 20.0 --m
    },
    line_seg_topic = "/line_markers",
    frequency = 10,
    use_particle_filter = true
}
return config