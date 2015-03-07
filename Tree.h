/*
 * Tree.h
 *
 *  Created on: 08-Dec-2014
 *      Author: Vemula
 */

#include <iostream>
using namespace std;

#ifndef TREE_H_
#define TREE_H_

namespace mytree {

// Define a structure to be used as the tree node
struct TreeNode
{
    int      Key;
    TreeNode *left;
    TreeNode *right;
    TreeNode *parent;
};

class Tree
{
    private:


    public:
		TreeNode *root;
	    TreeNode *addHere;
	    static int const fill = 	10000000;
	    static int const conflict = 10000001;
	    static int const dontcare = 10000002;
	    int *inputarray;
		Tree();
        ~Tree();
        bool isEmpty();
        TreeNode *SearchTree(int Key);
        void SearchRecursive(TreeNode *node);
        int Insert(TreeNode *newNode);
        int Insert(int Key, int value);
        int Delete(int Key);
        void PrintOne(TreeNode *T);
        void PrintTree();
        int AddToTree(TreeNode *T,int Key);
        void initialize(int inputs);
        void BackTrack(TreeNode *T);
        void KeyExists(int Key, TreeNode *T);
    private:
        void ClearTree(TreeNode *T);
        TreeNode *DupNode(TreeNode * T);
        void PrintAll(TreeNode *T);
};

} /* namespace mytree */

#endif /* TREE_H_ */
