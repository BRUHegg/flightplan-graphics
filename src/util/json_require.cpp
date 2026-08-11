#include "json_require.hpp"

#include <cassert>

#include <nlohmann/json.hpp>
#include <string>

namespace json_require {

RequirementTree::JsonRequirement::JsonRequirement(const std::string& nm,
                                                  nlohmann::json::value_t tp,
                                                  bool is_soft)
    : name_{nm}, type_{tp}, is_soft_{is_soft} {}

bool RequirementTree::JsonRequirement::Verify(
    const nlohmann::json& data) const {
  auto it = data.find(name_);
  if(it == data.end()) {
    return is_soft_;
  }
  if (it->type() != type_) {
    return false;
  }
  bool res = true;
  if (next_ != nullptr) {
    res &= next_->Verify(data);
  }
  if (inner_ != nullptr) {
    if (type_ == nlohmann::json::value_t::array) {
      for (std::size_t i = 0; i < it->size() && res; ++i) {
        res &= inner_->Verify(it->at(i));
      }
    } else {
      res &= inner_->Verify(*it);
    }
  }
  if(!res) {
    return false;
  }
  if(batch_predicate_ || type_ != nlohmann::json::value_t::array) {
    res &= predicate_(*it);
  } else {
    for (std::size_t i = 0; i < it->size() && res; ++i) {
      res &= predicate_(it->at(i));
    }
  }
  return res;
}

RequirementTree::Node::Node(JsonRequirement* ptr) : node_{ptr} {}

void RequirementTree::Node::SetPredicate(pred_type pred, bool is_batch) {
  assert(pred != nullptr);
  if(node_ != nullptr) {
    node_->predicate_ = pred;
    node_->batch_predicate_ = is_batch;
  }
}

void RequirementTree::Node::ResetPredicate() {
  if(node_ != nullptr) {
    node_->batch_predicate_ = false;
    node_->predicate_ = RequirementTree::JsonRequirement::Identity;
  }
}

bool RequirementTree::Node::Verify(const nlohmann::json& data) const {
  if(node_ == nullptr) {
    return true;
  }
  return node_->Verify(data);
}

RequirementTree::Node::operator bool() const {
  return node_ != nullptr;
}

RequirementTree::JsonRequirement* RequirementTree::Populate(
  JsonRequirement* dst) {
  if(dst == nullptr) {
    return nullptr;
  }
  JsonRequirement* cr = new JsonRequirement{*dst};
  cr->inner_ = Populate(dst->inner_);
  cr->next_ = Populate(dst->next_);
  return cr;
}

RequirementTree::JsonRequirement** RequirementTree::GetTargetPtr(
  JsonRequirement* node, bool adjacent) {
  assert(node != nullptr);
  if(adjacent) {
    return &node->next_;
  }
  return &node->inner_;
}

void RequirementTree::Destroy(JsonRequirement* curr) {
  if (curr == nullptr) {
    return;
  }
  Destroy(curr->inner_);
  Destroy(curr->next_);
  delete curr;
}

RequirementTree::RequirementTree(const RequirementTree& other) { 
  root_ = Populate(other.root_); 
}

RequirementTree& RequirementTree::operator=(const RequirementTree& other) {
  if (&other == this) {
    return *this;
  }
  RequirementTree tmp{other};
  swap(tmp, *this);
  return *this;
}

RequirementTree::RequirementTree(RequirementTree&& other) { 
  root_ = other.Release(); 
}

RequirementTree& RequirementTree::operator=(RequirementTree&& other) noexcept {
  if (&other == this) {
    return *this;
  }
  root_ = other.Release();
  return *this;
}

RequirementTree::Node RequirementTree::Add(
  const std::string& name, nlohmann::json::value_t val, bool is_soft) {
  JsonRequirement* new_req = new JsonRequirement{name, val, is_soft};
  if (root_ == nullptr) {
    root_ = new_req;
  } else {
    new_req->next_ = root_;
    root_ = new_req;
  }
  return new_req;
}

RequirementTree::Node RequirementTree::Add(
  Node parent, const std::string& name, 
  nlohmann::json::value_t val, bool is_soft, bool adjacent) {
  assert(parent);
  JsonRequirement** tgt_ptr = GetTargetPtr(parent.node_, adjacent);
  if (*tgt_ptr != nullptr) {
    return nullptr;
  }
  *tgt_ptr = new JsonRequirement{name, val, is_soft};
  return *tgt_ptr;
}

RequirementTree::Node RequirementTree::Add(
  const RequirementTree& other) {
  RequirementTree tmp{other};
  if (root_ == nullptr) {
    root_ = tmp.Release();
    return root_;
  }
  root_->next_ = tmp.Release();
  return root_->next_;
}

RequirementTree::Node RequirementTree::Add(
  Node parent, const RequirementTree& other, bool is_soft,
                     bool adjacent) {
  assert(parent);
  JsonRequirement** tgt_ptr = GetTargetPtr(parent.node_, adjacent);
  if (*tgt_ptr != nullptr) {
    return nullptr;
  }
  RequirementTree tmp{other};
  *tgt_ptr = tmp.Release();
  (*tgt_ptr)->is_soft_ = is_soft;
  return *tgt_ptr;
}

bool RequirementTree::Verify(const nlohmann::json& data) const {
  if (root_ == nullptr) {
    return true;
  }
  return root_->Verify(data);
}

RequirementTree::JsonRequirement* RequirementTree::Release() {
  JsonRequirement* ret = root_;
  root_ = nullptr;
  return ret;
}

void RequirementTree::Destroy() {
  Destroy(root_);
  root_ = nullptr;
}

RequirementTree::~RequirementTree() { 
  Destroy(); 
}
}  // namespace json_require
