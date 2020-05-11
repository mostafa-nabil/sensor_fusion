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

	void insertHelper(Node** node, int depth, std::vector<float> point, int id)
	{
		std::cout <<"depth "<<depth<<std::endl;

		if(*node == NULL)
		{
			*node = new Node(point,id);
			return;
		}
		else
		{
			 	if(0 == depth%2)
				{
					if(point[0] < (*node)->point[0])
					{
						insertHelper(&(*node)->left,depth+1,point,id);
					}
					else
					{
						insertHelper(&(*node)->right,depth+1,point,id);
					}
				}
				else
				{
					if(point[1] < (*node)->point[1])
					{
						insertHelper(&(*node)->left,depth+1,point,id);
					}
					else
					{
						insertHelper(&(*node)->right,depth+1,point,id);
					}
				}
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 		
		insertHelper(&root,0,point,id);		
	}

	float calcDistance(std::vector<float> point1, std::vector<float> point2)
	{
		float dx = point1[0] - point2[0];
		float dy = point1[1] - point2[1];
		float dist2 = (dx*dx) + (dy*dy);
		return sqrt(dist2);
	}

	void searchHelper(	Node** node, 
						std::vector<float> target, 
						float distanceTol,
						int depth,
						std::vector<int> &ids)
	{
		if(*node == NULL)
		{
			return;
		}
		float d = calcDistance(target, (*node)->point);
		if(d <= distanceTol)
		{
			ids.push_back((*node)->id);
			// searchHelper(&(*node)->left, target, distanceTol, depth+1,ids);
			// searchHelper(&(*node)->right, target, distanceTol, depth+1,ids);
			//return;
		}
		
		if (0==depth%2)
		{
			float dx = (*node)->point[0] - target[0];

			if(target[0]-distanceTol<(*node)->point[0])
			{
				searchHelper(&(*node)->left, target, distanceTol, depth+1,ids);
			}
			if(target[0]+distanceTol>(*node)->point[0])
			{
				searchHelper(&(*node)->right, target, distanceTol, depth+1,ids);
			}
			
		}
		else
		{
			float dy = (*node)->point[1] - target[1];

			if(target[1]-distanceTol<(*node)->point[1])
			{
				searchHelper(&(*node)->left, target, distanceTol, depth+1,ids);
			}
			if(target[1]+distanceTol>(*node)->point[1])
			{
				searchHelper(&(*node)->right, target, distanceTol, depth+1,ids);
			}	
		}
		
			
		
		

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(&root, target, distanceTol, 0,ids);


		return ids;
	}
	

};




