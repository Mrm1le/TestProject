/*
 * @Author: tianhaijian@immotors.com
 * @Date: 2024-02-21 09:48:08
 * @LastEditors: TianHaijian
 * @FilePath: /TestProject/test_tree/test.cpp
 * @Description:
 *
 * Copyright (c) 2024 by NVIDIA, All Rights Reserved.
 */

#include "../utils/time.h"
#include <algorithm>
#include <iostream>
#include <queue>
#include <vector>
constexpr bool enable_timer = true;

struct TreeNode {
  int val;
  TreeNode *left;
  TreeNode *right;
  TreeNode(int x) : val(x), left(nullptr), right(nullptr){};
};

// Breadth First Search
int maxDepthBFS(TreeNode *root) {
  if (!root)
    return 0;
  std::queue<TreeNode *> qtree;
  qtree.push(root);
  int depth = 0;
  while (!qtree.empty()) {
    int qsize = qtree.size();
    while (qsize-- > 0) {
      auto node = qtree.front();
      qtree.pop();
      if (node->left)
        qtree.push(node->left);
      if (node->right)
        qtree.push(node->right);
    }
    depth++;
  }
  return depth;
}

// Depth First Search
int maxDepthDFS(TreeNode *root) {
  if (!root)
    return 0;
  int left_depth = maxDepthDFS(root->left);
  int right_depth = maxDepthDFS(root->right);
  return std::max(left_depth, right_depth) + 1;
}

std::vector<int> preorderTraversal(TreeNode *root) {
  std::vector<int> ret;
  if (!root)
    return ret;
  ret.emplace_back(root->val);
  auto left_ret = preorderTraversal(root->left);
  ret.insert(ret.end(), left_ret.begin(), left_ret.end());
  auto right_ret = preorderTraversal(root->right);
  ret.insert(ret.end(), right_ret.begin(), right_ret.end());
  return ret;
}

std::vector<int> inorderTraversal(TreeNode *root) {
  std::vector<int> ret;
  if (!root)
    return ret;
  ret.emplace_back(root->val);
  auto left_ret = inorderTraversal(root->left);
  ret.insert(ret.begin(), left_ret.begin(), left_ret.end());
  auto right_ret = inorderTraversal(root->right);
  ret.insert(ret.end(), right_ret.begin(), right_ret.end());
  return ret;
}

std::vector<int> postorderTraversal(TreeNode *root) {
  std::vector<int> ret;
  if (!root)
    return ret;
  ret.emplace_back(root->val);
  auto right_ret = postorderTraversal(root->right);
  ret.insert(ret.begin(), right_ret.begin(), right_ret.end());
  auto left_ret = postorderTraversal(root->left);
  ret.insert(ret.begin(), left_ret.begin(), left_ret.end());
  return ret;
}

int main() {
  // 创建一个示例二叉树
  TreeNode *root = new TreeNode(1);
  root->left = new TreeNode(2);
  root->right = new TreeNode(3);
  root->left->left = new TreeNode(4);
  root->left->right = new TreeNode(5);
  root->right->left = new TreeNode(6);
  root->right->right = new TreeNode(7);

  // 使用广度优先搜索遍历二叉树并输出结果
  std::cout << "Breadth First Search traversal of Binary Tree: ";
  std::cout << maxDepthBFS(root) << std::endl;

  std::cout << "Depth First Search traversal of Binary Tree: ";
  utils::Timer<enable_timer> max_depth_timer("max_depth_timer");
  max_depth_timer.Tic();
  std::cout << maxDepthDFS(root) << std::endl;
  max_depth_timer.Toc();
  auto preorder_result = preorderTraversal(root);
  std::cout << "preorder traversal of Binary Tree: ";
  std::for_each(preorder_result.begin(), preorder_result.end(),
                [](int val) { std::cout << val << " "; });
  std::cout << std::endl;

  auto inorder_result = inorderTraversal(root);
  std::cout << "inorder traversal of Binary Tree: ";
  std::for_each(inorder_result.begin(), inorder_result.end(),
                [](int val) { std::cout << val << " "; });
  std::cout << std::endl;

  auto postorder_result = postorderTraversal(root);
  std::cout << "postorder traversal of Binary Tree: ";
  std::for_each(postorder_result.begin(), postorder_result.end(),
                [](int val) { std::cout << val << " "; });
  std::cout << std::endl;

  // 释放内存
  delete root->left->left;
  delete root->left->right;
  delete root->right->left;
  delete root->right->right;
  delete root->left;
  delete root->right;
  delete root;

  return 0;
}