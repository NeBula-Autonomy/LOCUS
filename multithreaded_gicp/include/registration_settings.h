#pragma once

enum class RegistrationMethod { GICP, NDT };

using EnumToStringRegistrationMethods =
    std::pair<std::string, RegistrationMethod>;

const std::vector<EnumToStringRegistrationMethods>
    EnumToStringRegistrationMethodsVector = {
        EnumToStringRegistrationMethods("gicp", RegistrationMethod::GICP),
        EnumToStringRegistrationMethods("ndt", RegistrationMethod::NDT)};
// TODO: maybe somehow varialbe template, but it's available from cpp17 i think
RegistrationMethod getRegistrationMethodFromString(const std::string& mode) {
  for (const auto& available_vlo : EnumToStringRegistrationMethodsVector) {
    if (mode == available_vlo.first) {
      return available_vlo.second;
    }
  }
  throw std::runtime_error("No such Registration mode!: " + mode);
}
