local arg = {...}
local config = arg[1]
local prefix = "/"
local tf_prefix = ""
if config ~= nil then
    prefix = prefix..config.prefix.."/"
    if config.prefix ~= "" then
        tf_prefix = config.prefix.."/"
    end
end

local dslam = {
    use_robust_kernel = true,
    odom_frame = tf_prefix.."odom",
    fixed_frame = tf_prefix.."map",
    base_frame = tf_prefix.."base_link",
    scan_topic= prefix.."scan",
    map_topic = prefix.."map",
    constraints_topic = prefix.."constraints",
    publish_graph = true,
    throttle_scans = 2.0,
    map_update_rate = 3.0,
    resolution = 0.05,
    use_scan_matching = true,
    use_scan_barycenter = true,
    minimum_travel_distance = 0.2, -- 0.2
    minimum_travel_heading = 30.0*math.pi/180.0, -- 0.17
    scan_buffer_size = 70,
    scan_buffer_maximum_scan_distance = 20,
    link_match_minimum_response_fine=0.8,
    link_scan_maximum_distance=10.0,
    loop_search_maximum_distance=5.0, -- 6
    do_loop_closing = true,
    loop_match_minimum_chain_size = 10.0,
    loop_match_maximum_variance_coarse=0.4,
    loop_match_minimum_response_coarse = 0.8, -- 0.5
    loop_match_minimum_response_fine=0.8, -- 0.7
    -- correlation
    correlation_search_space_dimension=0.3,
    correlation_search_space_resolution=0.01,
    correlation_search_space_smear_deviation=0.03,
    -- loop closing
    loop_search_space_dimension=8.0,
    loop_search_space_resolution=0.05,
    loop_search_space_smear_deviation=0.03, -- 0.025
    -- scan matching
    distance_variance_penalty=0.3,
    angle_variance_penalty=0.349, -- degree
    fine_search_angle_offset=0.00349,
    coarse_search_angle_offset=0.349,
    coarse_angle_resolution=0.0349,
    minimum_angle_penalty=0.9,
    minimum_distance_penalty=0.5,
    use_response_expansion=false
}

return dslam