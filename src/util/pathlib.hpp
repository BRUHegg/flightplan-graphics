#pragma once

#include <cstdint>

#include <string>

namespace pathlib {

void SetDefaultPathSep(char x);

class Path final {
  char separator_;
  std::string path_ = "";
  std::string extension_ = "";
  std::string entry_ = "";
  bool is_dir_ = true;

  void Init(const std::string& path, char path_sep);

public:
  void SetEntry(const std::string& entry);

  Path();

  explicit Path(const std::string& path);

  Path(const std::string& path, char path_sep);

  char GetSeparator() const noexcept;

  Path GetPath() const noexcept;

  const std::string& GetExtension() const noexcept;

  std::string& GetExtension() noexcept;

  std::string GetEntry() const noexcept;

  bool IsDirectory() const noexcept;

  std::string Get() const noexcept;

  friend Path operator+(const Path& path, const std::string& str);

  friend Path& operator+=(Path& path, const std::string& str);
};
} // namespace pathlib
