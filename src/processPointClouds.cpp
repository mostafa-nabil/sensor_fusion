// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered {new pcl::PointCloud<PointT>};
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(* cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};
    pcl::CropBox<PointT> region(true);

    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    pcl::CropBox<PointT> roof(true);
    std::vector<int> indices;

    roof.setMin(Eigen::Vector4f {-2.5,-2.5,-1,1});
    roof.setMax(Eigen::Vector4f {2.5,2.5,0,1});
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    typename pcl::PointIndices::Ptr pointIndices {new pcl::PointIndices};
    for (int index: indices)
    {
        pointIndices->indices.push_back(index);
    }

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(pointIndices);
    extract.setNegative(true);
    extract.filter(*cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    
    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>()};
    
    for (int index: inliers-> indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstaclesCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	typename pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
    // TODO:: Fill in this function to find inliers for the cloud.

    typename pcl::ModelCoefficients::Ptr modelCoefficients{new pcl::ModelCoefficients()}; //{new pcl::ModelCoefficients};
    typename pcl::SACSegmentation<PointT> seg;

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *modelCoefficients);
    if(inliers->indices.size()==0)
    {
        std::cout<<"could not estimate planar model for the given dataset" << std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneOwn(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    /*intialize plane parameters of the final plane eqaution*/
    
    //maximum number of inliers
    typename pcl::PointIndices::Ptr lastMaxInliers {new pcl::PointIndices()}; 
    //plane equation constants
	float bestLineA = 0;
	float bestLineB = 0;
	float bestLineC = 0;
	float bestLineD = 0;

    /*main ransac iteration*/
    for(int i = 0; i < maxIterations; i++)
    {
        /*A plane is defined by three points, choose three random
        points from the point cloud*/
        PointT point1 = cloud->points[rand()%cloud->points.size()];
		PointT point2 = cloud->points[rand()%cloud->points.size()];
		PointT point3 = cloud->points[rand()%cloud->points.size()];

        /*for simplicity of the equations, use x1, y1, x2,..,etc variables*/
        float x1,x2,x3,y1,y2,y3,z1,z2,z3;
        x1 = point1.x;
        x2 = point2.x;
        x3 = point3.x;
        y1 = point1.y;
        y2 = point2.y;
        y3 = point3.y;
        z1 = point1.z;
        z2 = point2.z;
        z3 = point3.z;

        /*calculate the coefficients of the plane equation*/
        float A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float D = -(A*x1 + B*y1 + C*z1);

        /*find the number of inliers in the plane by looping on every point
        and calculating the distance to the plane*/
	    typename pcl::PointIndices inliers;

        //TODO: do not calculate distances for the three random points
        for(int idx = 0; idx < cloud->points.size(); idx++)
        {
            PointT point = cloud->points[idx];
            float d = abs(A*point.x + B*point.y + C*point.z + D)/sqrt(A*A + B*B + C*C);

            /*check if distance is withing the tolerance*/
            if(d <= distanceThreshold)
            {
                inliers.indices.push_back(idx);
            }
            else
            {
                //point is an outlier -> do nothing
            }
            
        }

        /*if current number of inliers is greater than last maximum number 
        of inliers, assign the current as the maximum an save the plane coefficients*/

        if (inliers.indices.size() > lastMaxInliers->indices.size())
		{
			*lastMaxInliers = inliers;
			bestLineA = A;
			bestLineB = B;
			bestLineC = C;
			bestLineD = D;
		}
        else
        {
            //do nothing
        }
        

    }//end for(int i = 0; i < maxIterations; i++)


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    /*extract inliers and outliers from input point cloud*/
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(lastMaxInliers,cloud);
    return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //create the KD tree

    typename pcl::search::KdTree<PointT>::Ptr tree{new pcl::search::KdTree<PointT>};
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster{new pcl::PointCloud<PointT>()};

        for(int index: getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void Proximity(typename pcl::PointCloud<PointT>::Ptr cloud, Kdtree<PointT> &tree, int &id ,float &distanceTol, std::vector<bool> &processed, pcl::PointIndices &clusterIndices)
{
    processed[id] = true;
    clusterIndices.indices.push_back(id);
    std::vector<int> nearby = tree.search(cloud->points[id], distanceTol);
    for (int i = 0; i < nearby.size(); i++)
    {
        if(false == processed[nearby[i]])
        {
            Proximity(cloud, tree, nearby[i], distanceTol, processed, clusterIndices);
        }
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringOwn(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointIndices> clustersIndices;   

    /*create the KDtree*/
    Kdtree<PointT> tree;

    for (int i = 0; i < cloud->points.size(); i++)
    {
        tree.insert(cloud->points[i], i);
    }

    /*Euclidean clustering process*/
    //vector for knowing processed/unprocessed points
    std::vector<bool> processed;

    //mark all points as unprocessed
    for (int i = 0; i < cloud->points.size();i++)
    {
        processed.push_back(false);
    }

    for (int i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointIndices clusterIndices;   
        if(false == processed[i])
        {
            Proximity(cloud, tree, i, clusterTolerance, processed, clusterIndices);
        }
        clustersIndices.push_back(clusterIndices);
    }
    
    for(pcl::PointIndices getIndices: clustersIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster{new pcl::PointCloud<PointT>()};

        //filter clusters by size
        if(getIndices.indices.size() > minSize && getIndices.indices.size() < maxSize)
        {
            for(int index: getIndices.indices)
            {
                cloudCluster->points.push_back(cloud->points[index]);
            }
            
            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        }
        
        
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}