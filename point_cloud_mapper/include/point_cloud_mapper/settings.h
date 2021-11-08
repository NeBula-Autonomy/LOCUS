#pragma once

#include <point_cloud_mapper/PointCloudIkdTreeMapper.h>
#include <point_cloud_mapper/PointCloudMapper.h>
#include <point_cloud_mapper/PointCloudMultiThreadedMapper.h>

enum class MappingMethod {
  MAPPER,
  BUFFER_MAPPER,
  OCTOMAP_MAPPER,
  VDB_MAPPER,
  KFLANN,
  MULTI_THREADED_MAPPER,
  IKDTREE_MAPPER
};

using EnumToStringMappingMethods = std::pair<std::string, MappingMethod>;

const std::vector<EnumToStringMappingMethods> EnumToStringMappingMethodsVector =
    {EnumToStringMappingMethods("mapper", MappingMethod::MAPPER),
     EnumToStringMappingMethods("buffer_mapper", MappingMethod::BUFFER_MAPPER),
     EnumToStringMappingMethods("kflann", MappingMethod::KFLANN),
     EnumToStringMappingMethods("octomap_mapper",
                                MappingMethod::OCTOMAP_MAPPER),
     EnumToStringMappingMethods("vdb_mapper", MappingMethod::VDB_MAPPER),
     EnumToStringMappingMethods("multi_threaded_mapper",
                                MappingMethod::MULTI_THREADED_MAPPER),
     EnumToStringMappingMethods("ikdtree_mapper",
                                MappingMethod::IKDTREE_MAPPER)};
// TODO: maybe somehow varialbe template, but it's available from cpp17 i think
MappingMethod getCalibrationMethodFromString(const std::string& mode) {
  for (const auto& available_vlo : EnumToStringMappingMethodsVector) {
    if (mode == available_vlo.first) {
      return available_vlo.second;
    }
  }
  throw std::runtime_error("No such mapping mode!: " + mode);
}

IPointCloudMapper::Ptr mapperFabric(const std::string& mapping_method) {
  switch (getCalibrationMethodFromString(mapping_method)) {
  case MappingMethod::MAPPER: {
    ROS_INFO_STREAM("MappingMethod::MAPPER activated.");
    return std::make_shared<PointCloudMapper>();
  }
  case MappingMethod::MULTI_THREADED_MAPPER: {
    ROS_INFO_STREAM("MappingMethod::MULTI_THREADED_MAPPER activated.");
    return std::make_shared<PointCloudMultiThreadedMapper>();
  }
  case MappingMethod::IKDTREE_MAPPER: {
    ROS_INFO_STREAM("MappingMethod::IKDTREE_MAPPER activated.");
    return std::make_shared<PointCloudIkdTreeMapper>();
  }
  //  case MappingMethod::OCTOMAP_MAPPER: {
  //    ROS_INFO_STREAM("MappingMethod::OCTOMAP_MAPPER activated.");
  //    return std::make_shared<PointCloudOctomapMapper>();
  //  }
  //  case MappingMethod::VDB_MAPPER: {
  //    ROS_INFO_STREAM("MappingMethod::VDB_MAPPER activated.");
  //    return std::make_shared<PointCloudVDBMapper>();
  //  }
  //  case MappingMethod::KFLANN: {
  //    ROS_INFO_STREAM("MappingMethod::KFLANN acticated.");
  //    return std::make_shared<PointCloudFlannMapper>();
  //  }
  default:
    throw std::runtime_error("No such mapping mode or not implemented yet " +
                             mapping_method);
  }
}
