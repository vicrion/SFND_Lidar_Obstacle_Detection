/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

#include <spdlog/spdlog.h>


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

	~Node()
	{
		spdlog::info("Node={} DTOR.", id);
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
		spdlog::info("KD tree: dtor.");
		delete root;
	}

	void insert(const std::vector<float>& point, int id)
	{
		insert2d(root, 0, point, id);

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search2d(root, 0, target, distanceTol, ids);

		return ids;
	}
	
protected:
	void search2d(Node* current, int level, const std::vector<float>& target, const float distanceTol, std::vector<int>& ids)
	{
		if (!current) return;

		spdlog::info("Checking id={}.", current->id);

		if (isWithin(target, current->point, distanceTol)) {
			ids.push_back(current->id);
			spdlog::info("Pushing id={}.", current->id);
		}

		int coordinate = level%target.size();

		if ( target.at(coordinate) - distanceTol < current->point.at(coordinate) ){
			search2d(current->left, level+1, target, distanceTol, ids);
		}
		if ( target.at(coordinate) + distanceTol > current->point.at(coordinate) ){
			search2d(current->right, level+1, target, distanceTol, ids);
		}
	}

	bool isWithin(const std::vector<float>& target, const std::vector<float>& point, const float distanceTol)
	{
		assert(target.size() == point.size());
		auto dx = target.at(0) -point.at(0); 
		auto dy = target.at(1) - point.at(1);
		return (dx*dx + dy*dy) < (distanceTol*distanceTol);
	}

	void insert2d(Node*& node, int level, const std::vector<float>& point, int id)
	{
		if (node == nullptr)
		{
			spdlog::info("Inserting Node={}.", id);
			node = new Node(point, id);
		}
		else 
		{
			int idx = level % 2;
			assert(idx < int(point.size()));
			assert(idx < int(node->point.size()));

			if (point.at(idx) < node->point.at(idx)  )
			{
				insert2d(node->left, level+1, point, id);
			}
			else 
			{
				insert2d(node->right, level+1, point, id);
			}
		}
	}

};




