local config = {
    line_extraction = {
        scan_topic = "/scan",
        bearing_std_dev = 1e-3,
        range_std_dev = 0.02,
        least_sq_angle_thresh = 1e-4,
        least_sq_radius_thresh = 1e-4,
        max_line_gap = 0.4,
        min_line_length = 0.5,
        min_range = 0.4,
        min_split_dist = 0.05,
        outlier_dist = 0.05,
        min_line_points = 9.0
    },
    line_seg_topic = "/line_markers",
    frequency = 25
}
return config