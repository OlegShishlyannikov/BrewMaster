#ifndef BTREE_HPP
#define BTREE_HPP

#include <cstdint>
#include <cstdio>

// A btree node
struct btree_node_s {
  // Make the btree friend of this so that we can access private members of this
  // class in btree functions
  friend struct btree_s;

  btree_node_s(int32_t _t, bool _leaf); // Constructor

  // A utility function to insert a new key in the subtree rooted with
  // this node. The assumption is, the node must be non-full when this
  // function is called
  void insert_non_full(int32_t k);

  // A utility function to split the child y of this node. i is index of y in
  // child array C[].  The Child y must be full when this function is called
  void split_child(int32_t i, struct btree_node_s *y);

  // A function to traverse all nodes in a subtree rooted with this node
  void traverse();

  // A function to search a key in the subtree rooted with this node.
  btree_node_s *search(int32_t k); // returns nullptr if k is not present.

  // A wrapper function to remove the key k in subtree rooted with
  // this node.
  void remove(int32_t k);

  // A function to remove the key present in idx-th position in
  // this node which is a leaf
  void remove_from_leaf(int32_t idx);

  // A function to remove the key present in idx-th position in
  // this node which is a non-leaf node
  void remove_from_non_leaf(int32_t idx);

  // A function to get the predecessor of the key- where the key
  // is present in the idx-th position in the node
  int get_pred(int32_t idx);

  // A function to get the successor of the key- where the key
  // is present in the idx-th position in the node
  int get_succ(int32_t idx);

  // A function to fill up the child node present in the idx-th
  // position in the C[] array if that child has less than t-1 keys
  void fill(int32_t idx);

  // A function to borrow a key from the C[idx-1]-th node and place
  // it in C[idx]th node
  void borrow_from_prev(int32_t idx);

  // A function to borrow a key from the C[idx+1]-th node and place it
  // in C[idx]th node
  void borrow_from_next(int32_t idx);

  // A function to merge idx-th child of the node with (idx+1)th child of
  // the node
  void merge(int32_t idx);
  int32_t find_key(int32_t k);

private:
  int32_t *keys_;                  // An array of keys_
  int32_t t_;                      // Minimum degree (defines the range for number of keys_)
  struct btree_node_s **children_; // An array of child pointers
  int32_t n_;                      // Current number of keys_
  bool leaf_;                      // Is true when node is leaf. Otherwise false
};

// A BTree
struct btree_s {
  // Constructor (Initializes tree as empty)
  btree_s(int32_t _t) {
    root_ = nullptr;
    t_ = _t;
  }

  // The main function that inserts a new key in this B-Tree
  void insert(int32_t k);

  // function to traverse the tree
  void traverse() {
    if (root_ != nullptr)
      root_->traverse();
  }

  // function to search a key in this tree
  struct btree_node_s *search(int32_t k) {
    return (root_ == nullptr) ? nullptr : root_->search(k);
  }

  // The main function that removes a new key in thie B-Tree
  void remove(int32_t k);

private:
  struct btree_node_s *root_; // Pointer to root node
  int32_t t_;                 // Minimum degree
};

#endif /* BTREE_HPP */
