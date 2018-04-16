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
            line_scale = 0.06,
            max_dist = 8.0 -- max laser range
        },
        icp = {
            max_distance = 0.04, -- 5cm correspondence distance
            max_iteration = 100, -- maximum number of iterations 
            tf_epsilon = 1e-6, -- transformation epsilon
            eu_fitness = 1.0, -- the euclidean distance difference epsilon
            sample_fitness = 0.5--, -- 30 cm
            --translation_tolerance = 1e-6
        },
        pf = {
            MU_SYSTEM_NOISE_X = 0.0,
            MU_SYSTEM_NOISE_Y = 0.0,
            MU_SYSTEM_NOISE_THETA = 0.0,
            SIGMA_SYSTEM_NOISE_X = 4e-6,
            SIGMA_SYSTEM_NOISE_Y = 4e-6,
            SIGMA_SYSTEM_NOISE_THETA = 4e-5 ,-- 4*PI*PI/(180*180),
            MU_MEAS_NOISE_X = 0.0,
            MU_MEAS_NOISE_Y = 0.0,
            MU_MEAS_NOISE_THETA = 0.0,
            SIGMA_MEAS_NOISE_X = 2e-5,
            SIGMA_MEAS_NOISE_Y = 2e-5,
            SIGMA_MEAS_NOISE_THETA = 2e-5,-- 4*PI*PI/(120*120),
            PRIOR_MU_X = -0.01,
            PRIOR_MU_Y = 0.01,
            PRIOR_MU_THETA = PI/4,
            PRIOR_COV_X = 1e-4,
            PRIOR_COV_Y = 1e-4,
            PRIOR_COV_THETA = (PI/8)*(PI/8),
            NUM_SAMPLES = 100
        },
        keyframe_sample_linear = 0.7,
        keyframe_sample_angular = 0.9,
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
    frequency = 10,
    use_particle_filter = false
}
return config