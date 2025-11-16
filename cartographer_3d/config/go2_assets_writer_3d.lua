-- go2_assets_writer_3d.lua
-- 生成简洁的 3D 点云（PLY）

VOXEL_SIZE = 5e-2  -- 5 cm 体素

include "transform.lua"

options = {
  tracking_frame = "base_imu",  -- 必须和 go2_3d.lua 里一致
  pipeline = {
    -- 距离过滤：去掉太近/太远的点
    {
      action = "min_max_range_filter",
      min_range = 0.5,
      max_range = 30.,
    },

    -- 随机下采样，避免点太多（可以改成 1.0 关闭下采样）
    {
      action = "fixed_ratio_sampler",
      sampling_ratio = 0.2,
    },

    -- 直接写成 PLY 点云
    {
      action = "write_ply",
      filename = "points.ply",
    },
  }
}

return options
