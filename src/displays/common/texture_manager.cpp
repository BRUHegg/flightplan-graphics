#include "texture_manager.hpp"

#include <optional>
#include <string>
#include <unordered_map>

#include <cairo-ft.h>
#include <cairo.h>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <nlohmann/json.hpp>

#include <util/pathlib.hpp>

#include "cairo_utils.hpp"

namespace {

const char kMainFont[] = "main_font";
const char kFontType[] = "font";
const char kTextureType[] = "texture";
const char kFontDirName[] = "fonts";
const char kTextureDirName[] = "textures";
}

namespace fms_displays {

bool TextureManager::CheckMainFont(const nlohmann::json& data) noexcept {
  for(auto font_data: data) {
    if(font_data["name"] == std::string{kMainFont} && 
      font_data["type"] == std::string{kFontType}) {
      return true;
    }
  }
  return false;
}

bool TextureManager::CheckTextureType(const nlohmann::json& data) noexcept {
  if(data == std::string{kFontType} || 
    data == std::string{kTextureType}) {
    return true;
  }
  return false;
}

TextureManager::TextureManager(const pathlib::Path& base, const nlohmann::json& names,
  FT_Library* ft_lib) {
  pathlib::Path font_path = base + std::string{kFontDirName};
  pathlib::Path tex_path = base + std::string{kTextureDirName};
  for(auto texture_data: names) {
    font_data_t c_data;
    std::string base_str = font_path.Get();
    std::string tex_name = texture_data["name"];
    std::string file_name = texture_data["file_name"];
    if(texture_data["type"] == std::string{kFontType}) {
      bool res = cairo_utils::load_font(base_str + file_name,
        *ft_lib, &c_data.ft_face, &c_data.cairo_face);
      if(res) { 
        fonts_[tex_name] = c_data;
      }
    } else {
      pathlib::Path full_path = tex_path + file_name;
      cairo_surface_t* surf = cairo_image_surface_create_from_png(
        full_path.Get().c_str());
      if(surf != nullptr) {
        textures_[tex_name] = surf;
      }
    }
  }
}

std::optional<TextureManager::font_data_t> TextureManager::GetFontData(
  const std::string& font_name) const noexcept {
  auto it = fonts_.find(font_name);
  if(it == fonts_.end()) {
    return std::nullopt;
  }
  return it->second;
}

TextureManager::texture_t TextureManager::GetTexture(
  const std::string& tex_name) const noexcept {
  auto it = textures_.find(tex_name);
  if(it == textures_.end()) {
    return nullptr;
  }
  return it->second;
}

TextureManager::~TextureManager() {
  for(auto i: fonts_) {
    cairo_font_face_destroy(i.second.cairo_face);
    FT_Done_Face(i.second.ft_face);
  }
  for(auto i: textures_) {
    cairo_surface_destroy(i.second);
  }
}
} // namespace fms_displays
