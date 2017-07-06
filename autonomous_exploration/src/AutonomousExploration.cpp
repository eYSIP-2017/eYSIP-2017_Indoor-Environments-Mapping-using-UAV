#include <autonomous_exploration/AutonomousExploration.h>
#include <queue>

#define INF 0x7fffffff
using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;


namespace AutonomousExploration_server
{
    // parameters
	AutonomousExploration::AutonomousExploration(ros::NodeHandle private_nh_)
	: m_nh(),
	m_pointCloudSub(NULL),
	m_tfPointCloudSub(NULL),
	m_octree(NULL),
	m_maxRange(-1.0),
	m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
	m_res(0.05),
	m_treeDepth(0),
	m_maxTreeDepth(0),
	m_probHit(0.7), m_probMiss(0.4),
	m_thresMin(0.12), m_thresMax(0.97),
	m_pointcloudMinZ(-std::numeric_limits<double>::max()),
	m_pointcloudMaxZ(std::numeric_limits<double>::max()),
	m_occupancyMinZ(-std::numeric_limits<double>::max()),
	m_occupancyMaxZ(std::numeric_limits<double>::max()),
	m_compressMap(true),
	m_clusterCost(1.0),
	m_distanceCost(1.0),
	m_goalTolerance(0.0),
	lut(16)
	{
		ros::NodeHandle private_nh(private_nh_);
		private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
		private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);

		private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
		private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
		private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
		private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);

		private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);

		private_nh.param("resolution", m_res, m_res);
		private_nh.param("sensor_model/hit", m_probHit, m_probHit);
		private_nh.param("sensor_model/miss", m_probMiss, m_probMiss);
		private_nh.param("sensor_model/min", m_thresMin, m_thresMin);
		private_nh.param("sensor_model/max", m_thresMax, m_thresMax);
		private_nh.param("compress_map", m_compressMap, m_compressMap);
		private_nh.param("cluster_cost", m_clusterCost, m_clusterCost);
		private_nh.param("distance_cost", m_distanceCost, m_distanceCost);
		private_nh.param("goal_tolerance", m_goalTolerance, m_goalTolerance);

		// initialize octomap object & params
		m_octree = new OcTree(m_res);
		m_octree->setProbHit(m_probHit);
		m_octree->setProbMiss(m_probMiss);
		m_octree->setClampingThresMin(m_thresMin);
		m_octree->setClampingThresMax(m_thresMax);
		m_treeDepth = m_octree->getTreeDepth();
		m_maxTreeDepth = m_treeDepth;

		m_color.r = 0.0;
		m_color.g = 0.0;
		m_color.b = 1.0;
		m_color.a = 1.0;

		m_colorFree.r = 0.0;
		m_colorFree.g = 1.0;
		m_colorFree.b = 0.0;
		m_colorFree.a = 1.0;

		m_colorFrontier.r = 1.0;
		m_colorFrontier.g = 0.0;
		m_colorFrontier.b = 0.0;
		m_colorFrontier.a = 1.0;

		m_colorgoalFrontier.r = 1.0;
		m_colorgoalFrontier.g = 1.0;
		m_colorgoalFrontier.b = 0.0;
		m_colorgoalFrontier.a = 1.0;

		num_pcl = 0;
		m_deleteFlag = false;
        
        //publishers to visualise the diffferent types of cells
		m_goalposePub = m_nh.advertise<visualization_msgs::Marker>("frontier_goal_marker", 1, false);
		m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, false);
		m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, false);
		m_markerPubFro = m_nh.advertise<visualization_msgs::MarkerArray>("frontier_cells_vis_array", 1, false);
		m_goalPoint = m_nh.advertise<geometry_msgs::Pose>("/goal_autonomous", 1, false);
        
    	// subscribe to feedback from MoveIt!
		sub = m_nh.subscribe("/fbet/feedback", 1, &AutonomousExploration::feedback, this);
    
        // subscribe to pointcloud
		m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
		
		// subscribe to tf between world frame and base framee
		m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
		m_tfPointCloudSub->registerCallback(boost::bind(&AutonomousExploration::insertCallback, this, _1));	
	}
    
    //callback function that takes feedback from MoveIt! on whether the current goal is valid for path planning 
	void AutonomousExploration::feedback(const std_msgs::String::ConstPtr& data)
	{
		ROS_INFO("Deleting Key");
		OcTreeKey key;
		// check if the goal sent to MoveIt! still exists
		if(!m_octree->coordToKeyChecked(best_frontiergoal, key))
		{
			OCTOMAP_ERROR_STR("Error in search: [" << best_frontiergoal << "] is out of OcTree bounds!");
			return;
		}
		
		OcTreeKey temp;
		// store the goals coordinates
		if(m_octree->coordToKeyChecked(best_frontiergoal, temp))
		{
			// get the key to the goal coordinate in frontier_cells set 
			KeySet::iterator found = frontier_cells.find(temp);
			if(found != frontier_cells.end())
			{
			    // if the iterator exists, delete the key
				ROS_INFO("Deleted Key");
				frontier_cells.erase(temp);
			}
			else
			{
				ROS_INFO("Not Found");
			}
		}
		m_deleteFlag = true;
	}
    
    // destructor of the autonomous exploration class 
	AutonomousExploration::~AutonomousExploration()
	{
		if (m_tfPointCloudSub)
		{
			delete m_tfPointCloudSub;
			m_tfPointCloudSub = NULL;
		}

		if (m_pointCloudSub)
		{
			delete m_pointCloudSub;
			m_pointCloudSub = NULL;
		}

		if (m_octree)
		{
			delete m_octree;
			m_octree = NULL;
		}	
	}
    
	// function to find neighbor coordinates of a cell which has the key start_key
	void AutonomousExploration::genNeighborCoord(OcTreeKey start_key, std::vector<octomap::point3d>& occupiedNeighbor) 
	{
		
		// clear previous data
		occupiedNeighbor.clear();
	
		OcTreeKey neighbor_key;
		//checks all 26 possible neighbors of a cell in 3D space
		for (int i = 0; i < 26; i++) 
		{
			// fetch key of neighbor i
			lut.genNeighborKey(start_key, i, neighbor_key);
			
			// get the coordinate of the key of the neighbor
			point3d query = m_octree->keyToCoord(neighbor_key);
			
			// add the coordinate to the vector
			occupiedNeighbor.push_back(query);
		}
	}

    // finds the changed cells in the incoming point cloud and marks them as occupied or free
	void AutonomousExploration::trackChanges(pcl::PointCloud<pcl::PointXYZI>& changedCells) 
	{
		KeyBoolMap::const_iterator startPnt = m_octree->changedKeysBegin();
		KeyBoolMap::const_iterator endPnt = m_octree->changedKeysEnd();
        
        // for loop from the first to last cell in the octree
		for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) 
		{
		    // finds the node in the octree of the first element in the iterators
			OcTreeNode* node = m_octree->search(iter->first);
            
            // check if the node is occupied
			bool occupied = m_octree->isNodeOccupied(node);

			pcl::PointXYZI pnt;
	        
	        // find 3D coordinates
			pnt.x = m_octree->keyToCoord(iter->first.k[0]);
			pnt.y = m_octree->keyToCoord(iter->first.k[1]);
			pnt.z = m_octree->keyToCoord(iter->first.k[2]);
            
            // if occupied make intensity high
			if (occupied) 
			{
				pnt.intensity = 1000;
			}
			// if not occupied make intensity low
			else 
			{
				pnt.intensity = -1000;
			}
			
			// use only cells which have z coordinate less than the maximum allowed
			if(pnt.z <= m_pointcloudMaxZ)
			{
				changedCells.push_back(pnt);
			}
		}
		m_octree->resetChangeDetection();
	}

	// Function to apply the k-means algorithm on the frontier cells
	void AutonomousExploration::apply_kmeans(point3d sensorOrigin, std::vector<OcTreeKey>& frontier_cluster)
	{
		// The number of clusters 'k' depends upon of the number of frontier cells
		// 10 is the appropriate size of a cluster
		int num_k = frontier_cluster.size()/10 + 1;

		// The code below randomly chooses one of the frontier cells as the centroids for each cluster
		int num[num_k];
		OcTreeKey centroids[num_k];

		num[0] = (((double)rand()/RAND_MAX)*frontier_cluster.size());
		centroids[0] = frontier_cluster[num[0]];

		for(int i=1;i<num_k;i++)
		{
			num[i] = (((double)rand()/RAND_MAX)*frontier_cluster.size());
			centroids[i] = frontier_cluster[num[i]];

			// This is to prevent two clusters from having the same centroids
			for(int j=0;j<i;j++)
			{
				if(centroids[i]==centroids[j])
				{
					i--;
					break;
				}
			}
		}

		// means stores the centroids as points
		point3d means[num_k];
		for(int i=0;i<num_k;i++)
		{
			means[i] = m_octree->keyToCoord(centroids[i]);
		}

		// check_flag is a sentinel value to indicate that at least one of the clusters has the same mean as in the previous iteration
		bool check_flag = false;

		// array to store the prev_means
		point3d prev_means[num_k];

		// array to store new clusters formed
		std::vector<OcTreeKey> clusters[num_k];

		// this loop is executed until at least one of the clusters has the same mean as in the previous iteration
		do
		{

			// store the previous means and clear the clusters
			for(int i=0;i<num_k;i++)
			{
				prev_means[i] = means[i];
				clusters[i].clear();
			}

			// Iterate through all the frontier cells and find out which centroid is the closest to this cell
			for (std::vector<OcTreeKey>::iterator iter = frontier_cluster.begin(), end = frontier_cluster.end(); iter!=end; ++iter)
			{
				// convert the frontier cell to a co-ordinate in the world
				point3d point = m_octree->keyToCoord(*iter);
				double distances[num_k];
				double min_distance = INF;
				int min_i;

				// Find out which centroid is the closest to the current frontier cell
				for(int i=0;i<num_k;i++)
				{
					distances[i] = point.distance(means[i]);
					if(distances[i]<min_distance)
					{
						min_distance = distances[i];
						min_i = i;
					}
				}

				// insert the frontier cell into the closest cluster
				clusters[min_i].push_back(*iter);
			}

			// Find the centroid of the newly formed clusters if it has any frontier cells in it
			for(int i=0;i<num_k;i++)
			{
				if(clusters[i].size()>0)
				{
					find_center(clusters[i], centroids[i]);
					means[i] = m_octree->keyToCoord(centroids[i]);
				}
			}

			// Check if any of the cluster centroids are the same as in the previous iteration
			for(int i=0;i<num_k;i++)
			{
				if(means[i]==prev_means[i])
				{
					check_flag = true;
				}
			}
		}
		while(!check_flag);

		// Store the clusters and the centroids for further use
		for (int i = 0; i < num_k; ++i)
		{
			if(clusters[i].size()>0)
			{
				final_clusters.push_back(std::make_pair(clusters[i], centroids[i]));
				candidate_cells.insert(centroids[i]);
			}
		}
	}
    
    // checks if a cell is a frontier cell
	int AutonomousExploration::is_frontier(OcTreeKey& o_key)
	{
		std::vector<octomap::point3d> neighbor;
		int flag1, flag2;
		point3d point;
		//get coordinates of the key
		point = m_octree->keyToCoord(o_key);
		float res = m_res/2;
		float low = res;
		float high = m_pointcloudMaxZ - res;
		
		// coordinate is the lowest or the highest allowed then do nothing
		if(point.z()==low || point.z()==high)
			return 0;
		OcTreeKey key;
		
		key = o_key;
		
		// find the node in the octree of the key
		OcTreeNode* node = m_octree->search(key);
		
		// check if the node is occupied
		bool occupied = m_octree->isNodeOccupied(node);
		if(!occupied)
		{
			flag1=0;
			flag2=0;
			
			// get neighbors of the cell
			genNeighborCoord(key, neighbor) ;
			
			// iterate through the neighbors
			for (std::vector<point3d>::iterator iter = neighbor.begin();iter != neighbor.end(); iter++)
			{
				point3d neipoint=*iter;

				//check point state: free/unknown
				OcTreeNode* node = m_octree->search(neipoint);
				
				// if neighbor is unknown set flag1
				if(node == NULL)
				flag1=1;
				else
				{
				    //if nieghbor is free set flag2
					if(!m_octree->isNodeOccupied(node))
					flag2=1;
				}
			}
			
			// if atleast 1 neighbor is unknown and 1 neighbor is free, mark the cell as a frontier cell
			if(flag1==1 && flag2==1)
			{
				return 1;
			}
		}
		return 0;
	}
    
    //function to find frontier cells in an incoming pointcloud
	void AutonomousExploration::find_frontier(pcl::PointCloud<pcl::PointXYZI>& changedCells , KeySet& frontierCells)
	{
		int flag1,flag2,i;
		std::vector<octomap::point3d> neighbor;
		
		i = changedCells.points.size();
		
		// for loop to iterate through the changed cells
		for(i = 0; i < changedCells.points.size(); i++)
		{
			//get changed point
			float x,y,z;
			x = changedCells.points[i].x;
			y = changedCells.points[i].y;
			z = changedCells.points[i].z;
			
			point3d changed_point(x,y,z);
			//transform from point to key
			OcTreeKey key;
			if (!m_octree->coordToKeyChecked(changed_point, key)) 
			{
				// ROS_INFO("ERROR!!!");
				OCTOMAP_ERROR_STR("Error in search: [" << changed_point << "] is out of OcTree bounds!");
				return;
			}
			
			// if cell is a frontier cell then store it
		    if(is_frontier(key))
		    {
			    frontierCells.insert(key);
		    }
		}
	}

    // function to find center cell of a cluster
	void AutonomousExploration::find_center(std::vector<OcTreeKey>& cluster, OcTreeKey& centerCell)
	{
		octomap::point3d centroid;
		octomap::point3d curr;
		OcTreeKey min_cell;
		int count = 0;
		
		// iterate through a cluster
		for(vector<OcTreeKey>::iterator iter = cluster.begin(), end=cluster.end(); iter!=end; ++iter)
		{
			// convert key to 3D coordinates
			curr = m_octree->keyToCoord(*iter);
			
			// find sum of coodinates of all cells
			centroid += curr;
			count++;
		}
		
		//find the centroid of the cluster
		centroid /= count;
		
		float min_distance = INF, distance;
		
		// iterate through the cluster and find the nearest cell to the centroid of the cell
		for(vector<OcTreeKey>::iterator iter = cluster.begin(), end=cluster.end(); iter!=end; ++iter)
		{
			curr = m_octree->keyToCoord(*iter);
			
			// find distance from centroid of the current cell
			distance = curr.distance(centroid);
			
			// if distance is less than the minimum distance mark the cell as the center cell
			if(distance < min_distance)
			{
				min_distance = distance;
				min_cell = *iter;
			}
		}
		centerCell = min_cell;
	}
	
	
	// function to find goal cell from the center cell of all the clusters
	void AutonomousExploration::best_frontier(point3d cur_p)
	{
		float min_a=INF;
		float wei_l, wei_i, wei_a, best_i;
		int i;
		long max_size = 0, best_size;
		double min_distance = INF, best_distance;
		point3d best_goal;
		long sum_cluster = frontier_cells.size();

		// iterate through the clusters
		for(std::vector< std::pair< std::vector<OcTreeKey>, OcTreeKey > >::iterator iter = final_clusters.begin(), end=final_clusters.end(); iter!= end; ++iter)
		{
			point3d fpoint;
			std::pair< std::vector<OcTreeKey>, OcTreeKey > temp = *iter;
			
			// center cell of the cluster
			fpoint = m_octree->keyToCoord(temp.second);
			// store the cluster
			std::vector<OcTreeKey> temp_cluster = temp.first;
			
			// find distance between center cell and current position
			wei_l=(fpoint.x()-cur_p.x())*(fpoint.x()-cur_p.x())+(fpoint.y()-cur_p.y())*(fpoint.y()-cur_p.y())+(fpoint.z()-cur_p.z())*(fpoint.z()-cur_p.z());
			
			// if center cell is same as previous goal then delete that center cell from frontier cells
			if(fpoint.distance(best_frontiergoal)==0)
			{
				ROS_INFO("Deleted Best Frontier");
				m_deleteFlag = true;
			    frontier_cells.erase(temp.second);
			    continue;
			}
			
			// inversee of the number of frontier cells in the cluster
			wei_i = 1.0/(double)(temp.first.size());
			
			// total cost of the cluster based on distance to cluster and number of cells in the cluster
			wei_a = m_clusterCost*wei_i + m_distanceCost*wei_l;
			
			// if total cost is less than the minimum cost then mark the cluster as the best
			if (wei_a<min_a)
			{
				min_a=wei_a;
				best_goal=fpoint;
				best_distance = wei_l;
				best_size = temp.first.size();
				best_i = wei_i;
			}
		}
		
		// store the best center cell as the goal
		best_frontiergoal = best_goal;
		goal.position.x = best_goal.x();
		goal.position.y = best_goal.y();
		goal.position.z = best_goal.z();
		
		// make the goal z coordinate sufficently higher than the floor
		if(best_goal.z() < 0.2)
			goal.position.z = 0.2;
		
		// find slope of line between current position and goal and store that as the yaw angle goal 
		goal.orientation.x = atan((best_goal.y() - cur_p.y()) / (best_goal.x() - cur_p.x()));
		m_deleteFlag = false;
		
		// publish goal
		m_goalPoint.publish(goal);
	}

	void AutonomousExploration::insertCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
	{
		num_pcl++;

		PCLPointCloud pc; // input cloud for filtering and ground-detection
		pcl::fromROSMsg(*cloud, pc);

		tf::StampedTransform sensorToWorldTf;
		
		// find transform between world frame and camera frame
		try 
		{
			m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
		} 
		catch(tf::TransformException& ex)
		{
			ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
			return;
		}

		Eigen::Matrix4f sensorToWorld;
		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);


		// set up filter for height range, also removes NANs:
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setFilterFieldName("z");
		pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

		// directly transform to map frame:
		pcl::transformPointCloud(pc, pc, sensorToWorld);

		// just filter height range:
		pass.setInputCloud(pc.makeShared());
		pass.filter(pc);

        // call all required functions to add pointcloud to the octree and then find the next goal
		insertScan(sensorToWorldTf.getOrigin(), pc);
		publishAll(cloud->header.stamp);
		publishfrontier(cloud->header.stamp,frontier_cells, m_colorFrontier, m_markerPubFro);
		publishfrontiergoal(cloud->header.stamp);
        
        // re-publish goal every 5th point cloud
		if(num_pcl>5)
		{			
			m_goalPoint.publish(goal);
			num_pcl = 0;
		}
    }

	// this function is the core of the autonomous exploration node. All calculation happens through this function
	void AutonomousExploration::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& cloud)
	{
		// get the current location of the drone
		point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

		// Check if the current position of the drone is recorded in the octree
		if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)|| !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
		{
			ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
		}
		
		// variables to store free cells, occupied cells and the newly generated frontier cells
		KeySet free_cells, occupied_cells, temp_frontiercells;

		// Iterate through each point in the pointcloud and convert it into an octree key
		for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it)
		{
			point3d point(it->x, it->y, it->z);

			// check for the max-range of the camera
			if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) 
			{
				// Find free cells
				// The computeRayKeys function find a ray between two given points, thus accounting for all points between the drone and surfaces
				if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay))
				{
					for (KeyRay::iterator iter = m_keyRay.begin(), end=m_keyRay.end(); iter!= end; iter++) 
					{						
						// Delete all frontier cells that belong in this pointcloud
						// This is done to avoid overpopulating the map with frontier cells
						KeySet::iterator got = frontier_cells.find(*iter);
						if (got != frontier_cells.end())
						{
							frontier_cells.erase(*iter);
						}
					}
					// insert free cells
					free_cells.insert(m_keyRay.begin(), m_keyRay.end());  
				}

				// occupied cells are the surfaces in the pointcloud
				OcTreeKey key;
				if (m_octree->coordToKeyChecked(point, key))
				{
					occupied_cells.insert(key);
					updateMinKey(key, m_updateBBXMin);
					updateMaxKey(key, m_updateBBXMax);
				}
			}
			else 
			{
				// ray longer than maxrange
				point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
				if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay))
				{
					for (KeyRay::iterator iter = m_keyRay.begin(), end=m_keyRay.end(); iter!= end; iter++) 
					{
						// Delete all frontier cells that belong in this pointcloud
						// This is done to avoid overpopulating the map with frontier cells
						KeySet::iterator got = frontier_cells.find(*iter);
						if (got != frontier_cells.end())
						{
							frontier_cells.erase(*iter);
						}
					}
					free_cells.insert(m_keyRay.begin(), m_keyRay.end());

					octomap::OcTreeKey endKey;
					if (m_octree->coordToKeyChecked(new_end, endKey))
					{
						updateMinKey(endKey, m_updateBBXMin);
						updateMaxKey(endKey, m_updateBBXMax);
					} 
					else
					{
						ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
					}
				}
			}
		}

		// mark free cells only if not seen occupied in this cloud
		for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
		{
			if (occupied_cells.find(*it) == occupied_cells.end())
			{
				m_octree->updateNode(*it, false);
			}
		}

		// mark all occupied cells:
		for (KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) 
		{
			m_octree->updateNode(*it, true);
		}

		octomap::point3d minPt, maxPt;
 
		minPt = m_octree->keyToCoord(m_updateBBXMin);
		maxPt = m_octree->keyToCoord(m_updateBBXMax);
			 
		if (m_compressMap)
			m_octree->prune();

		// find the new cells inserted into the tree
		ROS_INFO("No. of Free Cells: %ld", free_cells.size());
		m_octree->enableChangeDetection(true);
		pcl::PointCloud<pcl::PointXYZI> changed_cells ;
		trackChanges(changed_cells);
		ROS_INFO("No. of Changed Cells: %ld", changed_cells.size());

		//for every cell in changed_cells check if the cell is a frontier cell
		find_frontier(changed_cells,temp_frontiercells);
		ROS_INFO("No. of New Frontier Cells: %ld", temp_frontiercells.size());
		for(KeySet::iterator iter = frontier_cells.begin(), end=frontier_cells.end(); iter!=end; ++iter)
		{
			OcTreeKey temp = *iter;
			if(!is_frontier(temp))
			{
				frontier_cells.erase(*iter);
			}
		}

		// add the newly created frontier cells to the global set of frontier cells
		for(KeySet::iterator iter = temp_frontiercells.begin(), end=temp_frontiercells.end(); iter!= end; ++iter)
		{
			frontier_cells.insert(*iter);
		}

		// execute if there are any frontier cells remaining
		if(frontier_cells.size()>0)
		{
			// compute a next goal only when the drone is at a distance of m_goalTolerance away from the previous goal or when the drone is unable to reach the goal
			if(best_frontiergoal.distance(sensorOrigin) < m_goalTolerance || m_deleteFlag)
			{
				std::vector<OcTreeKey> frontier_vector;
				for(KeySet::iterator iter = frontier_cells.begin(), end=frontier_cells.end(); iter!= end; ++iter)
				{
					frontier_vector.push_back(*iter);
				}

				// find new clusters
				if(frontier_vector.size()>1)
				{
					candidate_cells.clear();
					final_clusters.clear();
					apply_kmeans(sensorOrigin, frontier_vector);
				}
				// if only 1 frontier cell, that is our goal
				else if(frontier_vector.size()==1)
				{
					candidate_cells.clear();
					final_clusters.clear();
					KeySet::iterator iter = frontier_cells.begin();
					final_clusters.push_back(std::make_pair(frontier_vector, *iter));
					candidate_cells.insert(*iter);
				}

				ROS_INFO("Frontier Cells: %ld", frontier_cells.size());
				ROS_INFO("Clusters: %ld", final_clusters.size());
				ROS_INFO("Candidate Cells: %ld", candidate_cells.size());

				// Find the new goal point
				best_frontier(sensorOrigin);
			}
		}
		else
		{
			// If there are no frontier cells remaining, set the goal to the start point of the mapping session (0,0,0)
			goal.position.x = 0;
			goal.position.y = 0;
			goal.position.z = 0;
			goal.orientation.x = 0;
			point3d nullPoint(0 ,0, 0);
			best_frontiergoal = nullPoint;
		}
	}

	// This function is used to publish the frontier cells
	void AutonomousExploration::publishfrontier(const ros::Time& rostime, KeySet& frontierCells, std_msgs::ColorRGBA colorFrontier, ros::Publisher publisher)
	{
		// init markers for free space:
		visualization_msgs::MarkerArray frontierNodesVis;

		// each array stores all cubes of a different size, one for each depth level:
		frontierNodesVis.markers.resize(m_treeDepth+1);

		// Iterate through each frontier cell to convert it into a marker
		for(KeySet::iterator iter = frontierCells.begin(), end=frontierCells.end(); iter!= end; ++iter)
		{
			octomap::point3d fpoint;
			fpoint = m_octree->keyToCoord(*iter);
			geometry_msgs::Point cubeCenter;
			cubeCenter.x = fpoint.x();
			cubeCenter.y = fpoint.y();
			cubeCenter.z = fpoint.z();

			// limit the depth of the tree
			unsigned idx = 16;
			assert(idx < frontierNodesVis.markers.size());

			frontierNodesVis.markers[idx].points.push_back(cubeCenter);
		}

		// Add attributes to each frontier cell in the marker array
		for (unsigned i= 0; i < frontierNodesVis.markers.size(); ++i)
		{
			double size = m_octree->getNodeSize(i);

			frontierNodesVis.markers[i].header.frame_id = m_worldFrameId;
			frontierNodesVis.markers[i].header.stamp = rostime;
			frontierNodesVis.markers[i].ns = "map";
			frontierNodesVis.markers[i].id = i;
			frontierNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;

			// specify size of the frontier cells
			frontierNodesVis.markers[i].scale.x = size;
			frontierNodesVis.markers[i].scale.y = size;
			frontierNodesVis.markers[i].scale.z = size;

			// specify color of the frontier cells - red
			frontierNodesVis.markers[i].color = colorFrontier;

			// add only visible frontier cells
			if (frontierNodesVis.markers[i].points.size() > 0)
				frontierNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				frontierNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}

		// publish the frontier cells
		publisher.publish(frontierNodesVis);
	}

	// THis function is used to publish a marker for the goal that the drone has to navigate to
	void AutonomousExploration::publishfrontiergoal(const ros::Time& rostime)
	{
		visualization_msgs::Marker frontier_goal;

		// find the goal point
		geometry_msgs::Point cubeCenter;
		cubeCenter.x = best_frontiergoal.x();
		cubeCenter.y = best_frontiergoal.y();
		cubeCenter.z = best_frontiergoal.z();

		frontier_goal.points.push_back(cubeCenter);
		double size = m_octree->getNodeSize(m_treeDepth);

		// Add the attributes
		frontier_goal.header.frame_id = m_worldFrameId;
		frontier_goal.header.stamp = rostime;
		frontier_goal.ns = "map";
		frontier_goal.type = visualization_msgs::Marker::CUBE_LIST;

		// specify the size of the marker
		frontier_goal.scale.x = size;
		frontier_goal.scale.y = size;
		frontier_goal.scale.z = size;

		// specify the marker color - yellow
		frontier_goal.color = m_colorgoalFrontier;

		if (frontier_goal.points.size() > 0)
			frontier_goal.action = visualization_msgs::Marker::ADD;
		else
			frontier_goal.action = visualization_msgs::Marker::DELETE;

		// publish the marker
		m_goalposePub.publish(frontier_goal);
	}

	// This function is used to publish the free cells and occupied cells so that they can be visualized in RViz
	void AutonomousExploration::publishAll(const ros::Time& rostime)
	{
		size_t octomapSize = m_octree->size();
		if (octomapSize <= 1)
		{
			ROS_WARN("Nothing to publish, octree is empty");
			return;
		}

		// init markers for free space:
		visualization_msgs::MarkerArray freeNodesVis;

		// each array stores all cubes of a different size, one for each depth level:
		freeNodesVis.markers.resize(m_treeDepth+1);

		// init markers:
		visualization_msgs::MarkerArray occupiedNodesVis;

		// each array stores all cubes of a different size, one for each depth level:
		occupiedNodesVis.markers.resize(m_treeDepth+1);

		// now, traverse all leaf nodes in the tree:
		for (OcTree::iterator it = m_octree->begin(m_maxTreeDepth),end = m_octree->end(); it != end; ++it)
		{
			// check of the current cell is an occupied cell or not
			if (m_octree->isNodeOccupied(*it))
			{
				double z = it.getZ();
				// limit the cells to the z limits specified
				if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
				{
					double size = it.getSize();
					double x = it.getX();
					double y = it.getY();
								
					//create marker:
					unsigned idx = it.getDepth();
					assert(idx < occupiedNodesVis.markers.size());

					geometry_msgs::Point cubeCenter;
					cubeCenter.x = x;
					cubeCenter.y = y;
					cubeCenter.z = z;

					// add the marker to the array
					occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
				}
			} 
			// executed when the current cell is a free cell
			else
			{ 
				// node not occupied => mark as free in 2D map if unknown so far
				double z = it.getZ();
				if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
				{			
					double x = it.getX();
					double y = it.getY();

					//create marker for free space:
					unsigned idx = it.getDepth();
					assert(idx < freeNodesVis.markers.size());

					geometry_msgs::Point cubeCenter;
					cubeCenter.x = x;
					cubeCenter.y = y;
					cubeCenter.z = z;

					freeNodesVis.markers[idx].points.push_back(cubeCenter);
				}
			}
		}

		// Add the attributes of each occupied marker and publish the marker array
		for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i)
		{
			double size = m_octree->getNodeSize(i);

			// set the world frame id
			occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
			occupiedNodesVis.markers[i].header.stamp = rostime;
			occupiedNodesVis.markers[i].ns = "map";
			occupiedNodesVis.markers[i].id = i;
			occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;

			// set the size of the marker
			occupiedNodesVis.markers[i].scale.x = size;
			occupiedNodesVis.markers[i].scale.y = size;
			occupiedNodesVis.markers[i].scale.z = size;

			// set the color of the marker - blue
			occupiedNodesVis.markers[i].color = m_color;

			// add only the visible markers
			if (occupiedNodesVis.markers[i].points.size() > 0)
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		// publish the occupied cells
		m_markerPub.publish(occupiedNodesVis);

		// Add the attributes of each free marker and publish the marker array
		for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i)
		{
			double size = m_octree->getNodeSize(i);

			freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
			freeNodesVis.markers[i].header.stamp = rostime;
			freeNodesVis.markers[i].ns = "map";
			freeNodesVis.markers[i].id = i;
			freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;

			// set the size of the marker
			freeNodesVis.markers[i].scale.x = size;
			freeNodesVis.markers[i].scale.y = size;
			freeNodesVis.markers[i].scale.z = size;

			// set the color of the marker - green
			freeNodesVis.markers[i].color = m_colorFree;

			// add only the visible markers
			if (freeNodesVis.markers[i].points.size() > 0)
				freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}
		// publish the free cells
		m_fmarkerPub.publish(freeNodesVis);
	}
}