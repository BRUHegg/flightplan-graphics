#include <unordered_map>

namespace util
{
    struct enum_class_hash_t
    {
        template <typename T>
        std::size_t operator()(T t) const
        {
            return static_cast<size_t>(t);
        }
    };
}; // namespace util
