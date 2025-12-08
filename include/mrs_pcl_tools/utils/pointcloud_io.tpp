
template <typename PC_t>
bool mrs_pcl_tools::loadCloud(ILogger& logger, const std::string& filepath, typename std::shared_ptr<PC_t> const& cloud,
                              const bool verbose)
{
  bool success = false;

  if (filepath.length() >= 3)
  {
    if (0 == filepath.compare(filepath.length() - 3, 3, "pcd"))
    {
      INFO_LOG_COND(verbose, logger, "[PCLSupportLibrary] Reading PCD file from path " + filepath);
      success = pcl::io::loadPCDFile<typename PC_t::PointType>(filepath, *cloud) == 0;
    }
    else if (0 == filepath.compare(filepath.length() - 3, 3, "ply"))
    {
      INFO_LOG_COND(verbose, logger, "[PCLSupportLibrary] Reading PCD file from path " + filepath);
      success = pcl::io::loadPLYFile<typename PC_t::PointType>(filepath, *cloud) == 0;
    }
    else
    {
      ERROR_LOG_COND(verbose, logger, "[PCLSupportLibrary] Unknown format of cloud file: " + filepath);
    }
  }
  else
  {
    ERROR_LOG_COND(verbose, logger,
                   "[PCLSupportLibrary] Could not read cloud from file. Path not given in valid format: " + filepath);
  }

  INFO_LOG_COND(success && verbose, logger,
                "[PCLSupportLibrary] Loaded point cloud with " + std::to_string(cloud->points.size()) + " points.");

  return success;
}