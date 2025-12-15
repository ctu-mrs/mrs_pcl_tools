/*//{ getFieldOffset() */
template <typename pt_t>
std::tuple<bool, std::size_t> mrs_pcl_tools::getFieldOffset(const std::string& field_name)
{
  std::vector<pcl::PCLPointField> fields;
  const int field_idx = pcl::getFieldIndex<pt_t>(field_name, fields);

  if (field_idx == -1)
  {
    return {false, 0};
  }

  const std::size_t field_offset = fields.at(field_idx).offset;
  return {true, field_offset};
}
/*//}*/

/*//{ getFieldValue() */
template <typename T, typename pt_t>
T mrs_pcl_tools::getFieldValue(const pt_t& point, std::size_t field_offset)
{
  const std::uint8_t* pt_data = reinterpret_cast<const std::uint8_t*>(&(point));
  T field_value = 0;
  memcpy(&field_value, pt_data + field_offset, sizeof(T));
  return field_value;
}
/*//}*/