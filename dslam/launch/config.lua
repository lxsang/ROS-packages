local config = {
    scan_topic = "/scan",
    keypoint_topic = "keypoints",
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
            min_line_points = 9.0
        },
        icp = {
            max_distance = 0.1, -- 5cm correspondence distance
            max_iteration = 100, -- maximum number of iterations 
            tf_epsilon = 1e-6, -- transformation epsilon
            eu_fitness = 1 -- the euclidean distance difference epsilon
        }
    },
    line_seg_topic = "/line_markers",
    frequency = 25
}
return config