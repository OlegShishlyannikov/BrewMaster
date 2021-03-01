#ifndef FILE_HPP
#define FILE_HPP

#include "util/hash/fnv1a.hpp"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sys/types.h>

struct file_ops_s {
  const struct drv_model_cmn_s *owner;
  int32_t (*flock)();
  int32_t (*funlock)();
  int32_t (*open)(int32_t, mode_t);
  int32_t (*ioctl)(uint64_t, const void *, size_t);
  int32_t (*read)(void *const , size_t);
  int32_t (*write)(const void *, size_t);
  int32_t (*close)();
};

struct file_s {
  using this_ta = file_s;

  explicit file_s(const char *name, const struct file_ops_s *const ops)
      : ops(ops), name(name), name_hash(hash_64_fnv1a(name)), fd(0u), flags(0), oflags(0), mode(0u), left(nullptr),
        right(nullptr) {}
  virtual ~file_s() = default;

  const char *name;
  uint64_t name_hash;
  const struct file_ops_s *const ops;
  mutable int32_t fd;
  mutable int32_t oflags;
  mutable int32_t flags;
  mutable mode_t mode;
  this_ta *left, *right;
};

struct file_tree_s {
  explicit file_tree_s() noexcept : root_(nullptr), fd_cnt_(1u) {}
  virtual ~file_tree_s() noexcept {
    if (root_) {
      free_branch_(root_);
    }
  }

  struct file_s *add_file(const char *name, const struct file_ops_s *const ops) const noexcept {
    /* If root branch is empty */
    if (!root_) {
      root_ = insert_(root_, name, ops, fd_cnt_++);
      return root_;
    }

    /* Else insert in existing root branch */
    return insert_(root_, name, ops, fd_cnt_++);
  }

  struct file_s *find_file(const char *name, int32_t fd) const noexcept {
    const uint64_t name_hash = hash_64_fnv1a(name);
    return search_(root_, name_hash, fd);
  }

  struct file_s *find_file(const char *name) const noexcept {
    const uint64_t name_hash = hash_64_fnv1a(name);
    return search_(root_, name_hash);
  }

  struct file_s *find_file(int32_t fd) const noexcept {
    return search_(root_, fd);
  }

  struct file_s *remove_file(const char *name) const noexcept {
    const uint64_t name_hash = hash_64_fnv1a(name);
    struct file_s *file = search_(root_, name_hash);

    if (file) {
      return remove_(root_, name_hash, file->fd);
    }

    return file;
  }

  struct file_s *remove_file(const char *name, int32_t fd) const noexcept {
    const uint64_t name_hash = hash_64_fnv1a(name);
    return remove_(root_, name_hash, fd);
  }

  const char **create_file_list() const {
    size_t nfiles = 0u, current = 0u;
    struct file_s *current_file = root_, *pre_file;

    /* No root */
    if (!current_file)
      return nullptr;

    /* Count elements in binary tree */
    count_elements_(root_, &nfiles);

    /* Allocate memory for filename list */
    const char **list = reinterpret_cast<const char **>(malloc(nfiles * sizeof(const char *)));
    while (current_file != nullptr) {

      if (current_file->left == nullptr) {

        list[current++] = current_file->name;
        current_file = current_file->right;
      } else {

        /* Find the inorder predecessor of current */
        pre_file = current_file->left;
        while (pre_file->right != nullptr && pre_file->right != current_file)
          pre_file = pre_file->right;

        /* Make current as the right child of its inorder
           predecessor */
        if (pre_file->right == nullptr) {
          pre_file->right = current_file;
          current_file = current_file->left;
        }

        /* Revert the changes made in the 'if' part to restore
           the original tree i.e., fix the right child
           of predecessor */
        else {

          pre_file->right = nullptr;
          list[current++] = current_file->name;
          current_file = current_file->right;
        }
      }
    }

    return list;
  }

  void free_file_list(const char **list) const { free(list); }

private:
  mutable struct file_s *root_;
  mutable uint32_t fd_cnt_;

  void count_elements_(const struct file_s *const file, size_t *cnt) const {
    if (!file) {
      return;
    }

    (*cnt)++;
    count_elements_(file->left, cnt);
    count_elements_(file->right, cnt);
  }

  struct file_s *insert_(struct file_s *root, const char *name, const struct file_ops_s *const ops,
                         int32_t fd) const noexcept {
    if (root == nullptr) {
      struct file_s *file = new file_s(name, ops);
      file->fd = fd;
      return file;
    }

    if (fd < root->fd)
      root->left = insert_(root->left, name, ops, fd);
    else if (fd > root->fd)
      root->right = insert_(root->right, name, ops, fd);

    return root;
  }

  struct file_s *search_(struct file_s *branch, const uint64_t &name_hash, int32_t fd) const noexcept {
    if (branch == nullptr || (branch->fd == fd && branch->name_hash == name_hash))
      return branch;

    if (branch->fd < fd)
      return search_(branch->right, name_hash, fd);

    return search_(branch->left, name_hash, fd);
  }

  struct file_s *search_(struct file_s *branch, const uint64_t &name_hash) const noexcept {
    if (branch == nullptr || (branch->name_hash == name_hash))
      return branch;

    if (branch->name_hash != name_hash)
      return search_(branch->right, name_hash);

    return search_(branch->left, name_hash);
  }

  struct file_s *search_(struct file_s *branch, int32_t fd) const noexcept {
    if (branch == nullptr || (branch->fd == fd))
      return branch;

    if (branch->fd < fd)
      return search_(branch->right, fd);

    return search_(branch->left, fd);
  }

  struct file_s *remove_(struct file_s *branch, const uint64_t &name_hash, uint32_t fd) const noexcept {
    if (branch == nullptr)
      return branch;

    if (branch->fd > fd) {

      branch->left = remove_(branch->left, name_hash, fd);
      return branch;
    } else if (branch->fd < fd) {

      branch->right = remove_(branch->right, name_hash, fd);
      return branch;
    } else if (branch->fd == fd && branch->name_hash == name_hash) {
      if (branch->left == nullptr) {

        struct file_s *temp = branch->right;
        free(branch);
        return temp;
      } else if (branch->right == nullptr) {

        struct file_s *temp = branch->left;
        free(branch);
        return temp;
      } else {

        struct file_s *succ = branch->right;
        struct file_s *succ_parent = branch;

        while (succ->left != nullptr) {

          succ_parent = succ;
          succ = succ->left;
        }

        if (succ_parent != branch) {

          succ_parent->left = succ->right;
        } else {

          succ_parent->right = succ->right;
        }

        branch->fd = succ->fd;
        free(succ);
        return branch;
      }
    }

    return nullptr;
  }

  void free_branch_(struct file_s *file) const noexcept {
    if (!file)
      return;

    free_branch_(file->left);
    free_branch_(file->right);

    free(file);
  }
};

#endif /* FILE_HPP */
