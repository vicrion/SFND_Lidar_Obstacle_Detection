#ifndef KDTREE3D_H
#define KDTREE3D_H

#include <vector>
#include <algorithm>
#include <stack>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <spdlog/spdlog.h>

struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		if (left != nullptr) delete left;
		if (right != nullptr) delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

    template <typename TPoint>
    void insert(const typename pcl::PointCloud<TPoint>::Ptr cloud)
    {
		insertBalanced<TPoint>(cloud, 0, cloud->size() - 1, 0, root);

        // for (unsigned int i=0; i<cloud->size(); ++i){
        //     std::vector<float> point{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        //     insert(point, i);
        // }
    }

	void insert(const std::vector<float>& point, int id)
	{
		insertKd(root, 0, point, id);
	}

	template <typename TPoint>
	std::vector<int> search(const TPoint& target, float distanceTol)
	{
		std::vector<float> point{target.x, target.y, target.z};

		return search(point, distanceTol);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, float distanceTol)
	{
		std::vector<int> ids;
		searchKd(root, 0, target, distanceTol, ids);

		return ids;
	}
	
protected:
	void searchKd(Node* current, int level, const std::vector<float>& target, const float distanceTol, std::vector<int>& ids)
	{
		if (!current) return;

		spdlog::debug("Checking id={}.", current->id);

		if (isWithin(target, current->point, distanceTol)) {
			ids.push_back(current->id);
			spdlog::debug("Pushing id={}.", current->id);
		}

		int coordinate = level%target.size();

		if ( target.at(coordinate) - distanceTol < current->point.at(coordinate) ){
			searchKd(current->left, level+1, target, distanceTol, ids);
		}
		if ( target.at(coordinate) + distanceTol > current->point.at(coordinate) ){
			searchKd(current->right, level+1, target, distanceTol, ids);
		}
	}

	bool isWithin(const std::vector<float>& target, const std::vector<float>& point, const float distanceTol)
	{
		assert(target.size() == point.size());

        float dK = 0;
        for (unsigned int i=0; i<target.size(); ++i){
            dK += (target.at(i) - point.at(i)) * (target.at(i) - point.at(i)); 
        }

		return dK < (distanceTol*distanceTol);
	}

	template <typename TPoint>
	void insertBalanced(const typename pcl::PointCloud<TPoint>::Ptr cloud, int start, int end, int depth, Node*& node)
	{
		if (start > end) return;

		int axis = depth % 3; // Cycle through x, y, z axes
		int mid = start + (end - start) / 2;

		// Sort the range [start, end] by the selected axis
		std::sort(cloud->points.begin() + start, cloud->points.begin() + end + 1,
			[axis](const TPoint &a, const TPoint &b) {
				if (axis == 0) return a.x < b.x;
				else if (axis == 1) return a.y < b.y;
				else return a.z < b.z;
			});

		// Create a new node at the median point
		std::vector<float> point{cloud->points[mid].x, cloud->points[mid].y, cloud->points[mid].z};
		node = new Node(point, mid);

		// Recursively insert left and right halves
		insertBalanced<TPoint>(cloud, start, mid - 1, depth + 1, node->left);
		insertBalanced<TPoint>(cloud, mid + 1, end, depth + 1, node->right);
	}

	void insertKd(Node*& node, int level, const std::vector<float>& point, int id)
	{
		if (node == nullptr)
		{
			spdlog::debug("Inserting Node={}.", id);
			node = new Node(point, id);
		}
		else 
		{
			int idx = level % point.size();

			if (point.at(idx) < node->point.at(idx)  )
			{
				insertKd(node->left, level+1, point, id);
			}
			else 
			{
				insertKd(node->right, level+1, point, id);
			}
		}
	}

};

#endif // KDTREE3D_H