VOXEL_SIZE = 0.05

options = {
  tracking_frame = "body",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 0.3,
      max_range = 15.,
    },
    {
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = VOXEL_SIZE,
    },
    {
      action = "write_ply",
      filename = "carto_3d_map_v2.ply",
    },
  }
}

return options
