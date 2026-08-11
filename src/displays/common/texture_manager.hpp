#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include <cairo-ft.h>
#include <cairo.h>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <nlohmann/json.hpp>

#include <util/pathlib.hpp>

namespace fms_displays {

class TextureManager final {
public:
  struct font_data_t {
    cairo_font_face_t* cairo_face;
    FT_Face ft_face;
  };
private:

  std::unordered_map<std::string, font_data_t> fonts_;
  std::unordered_map<std::string, cairo_surface_t*> textures_;
public:
  static bool CheckMainFont(const nlohmann::json& data) noexcept;

  static bool CheckTextureType(const nlohmann::json& data) noexcept;

  using texture_t = cairo_surface_t*;

  TextureManager() = default;

  TextureManager(const pathlib::Path& base, const nlohmann::json& names, 
    FT_Library* ft_lib);

  TextureManager(const TextureManager& other) = delete;

  TextureManager(TextureManager&& other) = delete;

  TextureManager& operator=(const TextureManager& other) = delete;

  TextureManager& operator=(TextureManager&& other) = delete;

  std::optional<font_data_t> GetFontData(const std::string& font_name) const noexcept;

  texture_t GetTexture(const std::string& tex_name) const noexcept;

  ~TextureManager();
};
} // namespace fms_displays {