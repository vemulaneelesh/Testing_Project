/*
 * Tree.cpp
 *
 *  Created on: 08-Dec-2014
 *      Author: Vemula
 */

#include <iostream>
#include <string.h>
#include "Tree.h"

using namespace std;

namespace mytree {

Tree::Tree()
{
	root = new TreeNode();
	root->left = NULL;
	root->right = NULL;
	root->parent = NULL;
	return;

}

Tree::~Tree()
{
	ClearTree(root);
	return;
}

//--------------------------------------------
// Function: ClearTree()
// Purpose: Perform a recursive traversal of
//        a tree destroying all nodes.
//--------------------------------------------
void Tree::ClearTree(TreeNode *T)
{
    if(T==NULL) return;  // Nothing to clear
    if(T->left != NULL) ClearTree(T->left); // Clear left sub-tree
    if(T->right != NULL) ClearTree(T->right); // Clear right sub-tree
    delete T;    // Destroy this node
    return;
}

void Tree::BackTrack(TreeNode *node)
{
	//SearchRecursive(root);
	node->Key = conflict;
	int tempkey = node->parent->Key;
	bool rightnotempty = node->parent->right !=NULL? 1:0;
	bool leftnotempty = node->parent->left !=NULL? 1:0;
	int check = rightnotempty & leftnotempty;
	if(check)
	{
		if((node->parent->right->Key == conflict) && (node->parent->left->Key == conflict))
		{
			//BackTrack(node->parent);
			//inputarray[tempkey] = dontcare;
		}
	}
	else
	{
			AddToTree(node->parent,tempkey);
	}
	//In case if we forget what pis to reset, remember while backtracking what to reset after both children are conflicts we can reset
}

void Tree::SearchRecursive(TreeNode *node)
{
	if(node!=NULL)
	{
		if(node->Key == fill)
		{
			addHere = node;
		}
		else
		{
			SearchRecursive(node->left);
			SearchRecursive(node->right);
		}
	}
	else
	{
		//addHere = NULL;
	}
}
/*

void Tree::KeyExists(int key,TreeNode *node)
{
	if(node!=NULL)
	{
		if(node->Key == key)
		{
			exists = true;
		}
		else
		{
				KeyExists(key,node->left);
				KeyExists(key,node->right);
		}
	}
	else
	{
			exists = false;
	}
}
*/

int Tree::AddToTree(TreeNode *parentNode, int Key)
{
	TreeNode *childNode;

	childNode = new TreeNode();
	childNode->Key = fill;
	childNode->left = childNode->right = NULL;
	childNode->parent = parentNode;
	if(parentNode->left ==NULL || parentNode->left->Key != conflict)
	{
		    parentNode->left = childNode;
			childNode->parent = parentNode;
			parentNode->Key = Key;
			inputarray[Key] = 0;
			return 1;
	}
	else if(parentNode->right ==NULL || parentNode->right->Key != conflict)
	{
		    parentNode->right = childNode;
			childNode->parent = parentNode;
			parentNode->Key = Key;
			inputarray[Key] = 1;
			return 1;
	}
	else
	{
			return 0;
	}
}

void Tree::initialize(int inputs)
{
	root->Key = fill;
	inputarray= new int[inputs];
	for(int i=0;i<=inputs;i++)
	{
		inputarray[i] = dontcare;
	}

}

} /* namespace mytree */
