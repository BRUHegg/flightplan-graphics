#include "pathlib.hpp"

#include <cstdint>
#include <string>

namespace {

char glob_default_path_sep = '/';
} // namespace

namespace pathlib {

void SetDefaultPathSep(char x) {
  glob_default_path_sep = x;
}

void Path::Init(const std::string& path, char path_sep) {
  std::int64_t i = path.length();
  if (!i) {
    return;
  }
  --i;
  while (i >= 0 && path[i] != path_sep) {
    --i;
  }
  if (i > 0) {
    path_ = path.substr(0, i);
  }
  SetEntry(path.substr(i + 1));
}

void Path::SetEntry(const std::string& entry) {
  std::int64_t j = entry.size();
  if (!j) {
    extension_ = "";
    entry_ = "";
    is_dir_ = true;
    return;
  }
  --j;
  while (j >= 0 && entry[j] != '.') {
    --j;
  }
  if (j > 0) {
    std::string act_entry = entry.substr(0, j);
    std::string act_ext = entry.substr(j + 1);
    entry_ = act_entry;
    extension_ = act_ext;
    is_dir_ = false;
  } else {
    entry_ = entry;
    extension_ = "";
    is_dir_ = true;
  }
}

Path::Path() {
  separator_ = glob_default_path_sep;
}

Path::Path(const std::string& path) {
  separator_ = glob_default_path_sep;
  Init(path, glob_default_path_sep);
}

Path::Path(const std::string& path, char path_sep) : separator_{path_sep} {
  Init(path, path_sep);
}

char Path::GetSeparator() const noexcept { return separator_; }

Path Path::GetPath() const noexcept { return Path{path_, separator_}; }

const std::string& Path::GetExtension() const noexcept { return extension_; }

std::string& Path::GetExtension() noexcept { return extension_; }

std::string Path::GetEntry() const noexcept { return entry_; }

bool Path::IsDirectory() const noexcept { return is_dir_; }

std::string Path::Get() const noexcept {
  std::string res = path_ + std::string(1, separator_) + entry_;
  if (!is_dir_) {
    res += "." + extension_;
  }
  return res;
}

Path operator+(const Path& path, const std::string& str) {
  Path tmp{path};
  if (!path.IsDirectory()) {
    return tmp;
  }
  tmp.path_ += std::string(1, path.separator_) + path.entry_;
  tmp.SetEntry(str);
  return tmp;
}

Path& operator+=(Path& path, const std::string& str) {
  path = path + str;
  return path;
}
}  // namespace pathlib
