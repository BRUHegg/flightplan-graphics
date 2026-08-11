#pragma once

#include <string>
#include <nlohmann/json.hpp>

namespace json_require {

class RequirementTree final {
public:
  class Node;

  class JsonRequirement final {
    static bool Identity(const nlohmann::json&) {
      return true;
    }

    std::string name_ = "";
    nlohmann::json::value_t type_ = nlohmann::json::value_t::null;
    bool is_soft_ = false;
    bool batch_predicate_ = false;
    bool (*predicate_)(const nlohmann::json&) = Identity;
    JsonRequirement* next_ = nullptr;
    JsonRequirement* inner_ = nullptr;

    friend class RequirementTree;
    friend class RequirementTree::Node;

  public:
    JsonRequirement() = default;

    JsonRequirement(const std::string& nm, nlohmann::json::value_t tp, bool is_soft);

    bool Verify(const nlohmann::json& data) const;
  };
private:

  JsonRequirement* root_ = nullptr;

  static JsonRequirement* Populate(JsonRequirement* dst);

  static JsonRequirement** GetTargetPtr(
    JsonRequirement* node, bool adjacent);

  void Destroy(JsonRequirement* curr);

public:
  class Node final {
    JsonRequirement* node_ = nullptr;

    friend class RequirementTree;
  public:
    typedef bool (*pred_type)(const nlohmann::json&);

    Node() = default;

    Node(JsonRequirement* ptr);

    void SetPredicate(pred_type pred, bool is_batch);

    void ResetPredicate();

    bool Verify(const nlohmann::json& data) const; 

    operator bool() const;
  };

  RequirementTree() = default;

  RequirementTree(const RequirementTree& other);

  friend void swap(RequirementTree& tr_a, RequirementTree& tr_b) {
    std::swap(tr_a.root_, tr_b.root_);
  }

  RequirementTree& operator=(const RequirementTree& other);

  RequirementTree(RequirementTree&& other);

  RequirementTree& operator=(RequirementTree&& other) noexcept;

  Node Add(const std::string& name, nlohmann::json::value_t val,
    bool is_soft=false);

  Node Add(Node parent, const std::string& name, 
    nlohmann::json::value_t val, bool is_soft=false, bool adjacent=false);

  Node Add(const RequirementTree& other);

  Node Add(Node parent, const RequirementTree& other, 
    bool is_soft=false, bool adjacent=false);

  bool Verify(const nlohmann::json& data) const;

  JsonRequirement* Release();

  void Destroy();

  ~RequirementTree();
};
} // namespace json_require
