/*pcl includes*/
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

/*define Kdtree node*/
template <typename PointT>
struct Node
{
    PointT point;  //point data
    int id;        //id of the point  
    Node* left;     //pointer to left node
    Node* right;    //pointer to right node

    Node(PointT point_i, int id_i):point(point_i), id(id_i), left(NULL), right(NULL)
    {}
};

/*define Kdtree*/

template <typename PointT>
struct Kdtree
{

    Node<PointT>* root; //root node

    Kdtree(): root(NULL)
    {}

    //template <typename PointT>
    void insertHelper(Node<PointT>** node, int depth, PointT point, int id)
    {
        /*if no node is present -> insert a node*/
        if(*node == NULL)
        {
            *node = new Node<PointT>(point,id);
            return;
        }
        else
        {
            /*compare value depending on depth*/
            //first depth -> compare x
            if(0 == depth%3)
            {
                if(point.x < (*node)->point.x)
                {
                    insertHelper(&(*node)->left,depth+1,point,id);
                }
                else
                {
                    insertHelper(&(*node)->right,depth+1,point,id);
                }
            }
            //second depth -> compare y
            else if(1 == depth%3)
            {
                if(point.y < (*node)->point.y)
                {
                    insertHelper(&(*node)->left,depth+1,point,id);
                }
                else
                {
                    insertHelper(&(*node)->right,depth+1,point,id);
                }
            }
            //third depth -> compare z
            else
            {
                if(point.z < (*node)->point.z)
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

    // template <typename PointT>
    void insert(PointT point, int id)
    {
        /*call insertHelper to insert the point recursivelly*/
        insertHelper(&root,0,point,id);		
    }

    // template <typename PointT>
    float calcDistance(const PointT &point1, const PointT &point2)
	{
		float dx = point1.x - point2.x;
		float dy = point1.y - point2.y;
        float dz = point1.z - point2.z;
		float dist2 = (dx*dx) + (dy*dy) + (dz*dz);
		return sqrt(dist2);
	}

    // template <typedef PointT>
    void searchHelper(	Node<PointT>** node, 
						PointT target, 
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
		}
		
		if (0==depth%3)
		{
			float dx = (*node)->point.x - target.x;

			if(target.x-distanceTol<(*node)->point.x)
			{
				searchHelper(&(*node)->left, target, distanceTol, depth+1,ids);
			}
			if(target.x+distanceTol>(*node)->point.x)
			{
				searchHelper(&(*node)->right, target, distanceTol, depth+1,ids);
			}
			
		}
		else if(1==depth%3)
		{
			float dy = (*node)->point.y - target.y;

			if(target.y-distanceTol<(*node)->point.y)
			{
				searchHelper(&(*node)->left, target, distanceTol, depth+1,ids);
			}
			if(target.y+distanceTol>(*node)->point.y)
			{
				searchHelper(&(*node)->right, target, distanceTol, depth+1,ids);
			}	
		}
        else
        {
            float dz = (*node)->point.z - target.z;

			if(target.z-distanceTol<(*node)->point.z)
			{
				searchHelper(&(*node)->left, target, distanceTol, depth+1,ids);
			}
			if(target.z+distanceTol>(*node)->point.z)
			{
				searchHelper(&(*node)->right, target, distanceTol, depth+1,ids);
			}	
        }
        
		
			
		
		

	}

	// return a list of point ids in the tree that are within distance of target
    // template <typedef PointT>
	std::vector<int> search(PointT point, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(&root, point, distanceTol, 0,ids);


		return ids;
	}



};


