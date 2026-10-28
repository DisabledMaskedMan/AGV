#include "scan_core.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <system_error>
#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace scan_web {
void Config::save(const std::string & path) const {
  validate();
  const auto target = std::filesystem::canonical(path);
  auto document = YAML::LoadFile(target.string());
  auto parameters = document["mid360_suspended_object_detection"]["ros__parameters"];
  if (!parameters.IsMap()) throw std::invalid_argument("配置缺少 ros__parameters");
  for (const auto & field : object()) {
    const auto key = std::string(field.key());
    if (key == "min_cluster_points" || key == "max_cluster_points") parameters[key] = field.value().to_number<int>();
    else parameters[key] = field.value().to_number<double>();
  }
  YAML::Emitter emitter;
  emitter << document;
  if (!emitter.good()) throw std::runtime_error("无法序列化 YAML");
  const std::string content = std::string(emitter.c_str()) + '\n';
  std::filesystem::path temporary;
#ifdef _WIN32
  wchar_t name[MAX_PATH];
  if (!GetTempFileNameW(target.parent_path().c_str(), L"scw", 0, name))
    throw std::system_error(GetLastError(), std::system_category(), "无法创建临时 YAML");
  temporary = name;
  try {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    output.exceptions(std::ios::badbit | std::ios::failbit);
    output << content; output.close();
    if (!MoveFileExW(temporary.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
      throw std::system_error(GetLastError(), std::system_category(), "无法替换 YAML");
  } catch (...) {std::error_code ignored; std::filesystem::remove(temporary, ignored); throw;}
#else
  auto pattern = (target.parent_path() / ".scan-web-XXXXXX").string();
  std::vector<char> name(pattern.begin(), pattern.end()); name.push_back('\0');
  int descriptor = mkstemp(name.data());
  if (descriptor < 0) throw std::system_error(errno, std::generic_category(), "无法创建临时 YAML");
  temporary = name.data();
  try {
    struct stat metadata;
    if (stat(target.c_str(), &metadata) || fchmod(descriptor, metadata.st_mode & 0777))
      throw std::system_error(errno, std::generic_category(), "无法保留 YAML 权限");
    std::size_t offset = 0;
    while (offset < content.size()) {
      const auto written = write(descriptor, content.data() + offset, content.size() - offset);
      if (written < 0 && errno == EINTR) continue;
      if (written <= 0) throw std::system_error(errno, std::generic_category(), "无法写入 YAML");
      offset += static_cast<std::size_t>(written);
    }
    if (fsync(descriptor)) throw std::system_error(errno, std::generic_category(), "无法刷新 YAML");
    const int closed = close(descriptor); descriptor = -1;
    if (closed) throw std::system_error(errno, std::generic_category(), "无法关闭 YAML");
    std::filesystem::rename(temporary, target);
  } catch (...) {
    if (descriptor >= 0) close(descriptor);
    std::error_code ignored; std::filesystem::remove(temporary, ignored); throw;
  }
#endif
}
}
