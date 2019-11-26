/* \author Aaron Brown */
// Quiz on implementing kd tree

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
		//* If node is empty create the node with the point *//
		if (*node == NULL) 
		{
			*node = new Node (point, id);
			//std::cout << "node " << id << " inserted at depth " << depth << std::endl;
		}
		//* Otherwise call the recursive function again with left or right node pointer depending on the point dimension value *//
		else if (point[depth%3] < (*node)->point[depth%3]) 
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
		//* Call recursive function that inserts the point at the next empty node at the bottom of the tree. Start checking from root *//
		insertRecursive(&root,0,point,id);		
	}

	void searchRecursive(Node* node, uint depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			Eigen::Vector3f nodePoint(node->point[0],node->point[1],node->point[2]);
			Eigen::Vector3f targetPoint(target[0],target[1],target[2]);
			
			//* Check if distance is smaller than tolerance *//
			if ((targetPoint-nodePoint).norm()<=distanceTol)
			{
				ids.push_back(node->id);
				//std::cout<<"Found nearby node " << node->id << " at {" << node->point[0] << "," << node->point[1] << "}" << std::endl;
			}
			//* Only need to search further left-right if the target value is within the distance tolerance of the current node
			if ((target[depth%3]-distanceTol)<node->point[depth%3])
				searchRecursive(node->left,depth+1,target,distanceTol,ids);
			if ((target[depth%3]+distanceTol)>node->point[depth%3])
				searchRecursive(node->right,depth+1,target,distanceTol,ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		//* Create the return vector. Call recursive search function that will add to the vector the points that are within distance tolerance to the target *//
		std::vector<int> ids;
		searchRecursive(root,0,target,distanceTol,ids);
		return ids;
	}
	

};




