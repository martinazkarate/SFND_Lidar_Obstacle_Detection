/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertRecursive(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
		{
			*node = new Node (point, id);
			std::cout<<"node "<<id<<" inserted at depth "<<depth<<std::endl;
		}
		else if (point[depth%2] < (*node)->point[depth%2])
		{
			insertRecursive (&((*node)->left),depth+1,point,id);			
		}
		else
		{
			insertRecursive (&((*node)->right),depth+1,point,id);
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertRecursive(&root,0,point,id);		
	}

	void searchRecursive(Node* node, uint depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if (sqrt((target[0]-node->point[0])*(target[0]-node->point[0])+(target[1]-node->point[1])*(target[1]-node->point[1]))<=distanceTol)
			{
				ids.push_back(node->id);
				std::cout<<"Found nearby node " << node->id << " at {" << node->point[0] << "," << node->point[1] << "}" << std::endl;
			}				

			if ((target[depth%2]-distanceTol)<node->point[depth%2])
				searchRecursive(node->left,depth+1,target,distanceTol,ids);
			if ((target[depth%2]+distanceTol)>node->point[depth%2])
				searchRecursive(node->right,depth+1,target,distanceTol,ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRecursive(root,0,target,distanceTol,ids);
		return ids;
	}
	

};




