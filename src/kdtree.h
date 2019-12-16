/* \author Aaron Brown */
// Quiz on implementing kd tree

#pragma once

#include <math.h>
#include "pcl/PointIndices.h"

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
	PointT point;
	int id;
	Node *left;
	Node *right;

	Node(PointT p, int setId)
			: point(p), id(setId), left(NULL), right(NULL)
	{
	}
};

template <typename PointT>
struct KdTree
{
	typedef std::shared_ptr<KdTree<PointT>> Ptr;
	typedef std::shared_ptr<KdTree<PointT> const> ConstPtr;

	Node<PointT> *root;

	KdTree()
			: root(NULL)
	{
	}

	void insert_recursive(Node<PointT> *&node, PointT &point, int &id, size_t level)
	{
		if (node == NULL)
		{
			node = new Node<PointT>(point, id);
			return;
		}
		size_t index = level % 3;
		if (point.data[index] < node->point.data[index])
			insert_recursive(node->left, point, id, level+1 );
		else
			insert_recursive(node->right, point, id, level+1);
	}

	void insert(PointT point, int id)
	{
		// the function should create a new node and place correctly with in the root
		insert_recursive(root, point, id, 0);
	}

	void search_recursive(std::vector<int> &buf,
												PointT &target,
												float distanceTol,
												Node<PointT> *&node,
												size_t level)
	{
		if (node == NULL)
		{
			return;
		}
		size_t index = level % 3;

		// if point is in the box
		if (std::fabs(node->point.data[0] - target.data[0]) < distanceTol &&
				std::fabs(node->point.data[1] - target.data[1]) < distanceTol && 
				std::fabs(node->point.data[2] - target.data[2]) < distanceTol)
		{
			float dist = std::sqrt((node->point.data[0] - target.data[0]) * (node->point.data[0] - target.data[0]) +
														 (node->point.data[1] - target.data[1]) * (node->point.data[1] - target.data[1]));
			buf.push_back(node->id);
		}

		if (node->point.data[index] > (target.data[index] - distanceTol))
			search_recursive(buf, target, distanceTol, node->left, level + 1);

		if (node->point.data[index] < (target.data[index] + distanceTol))
			search_recursive(buf, target, distanceTol, node->right, level + 1);
	}

	// return a list of point ids in the tree that are within distance of target
	pcl::PointIndices::Ptr search(PointT &target, float distanceTol)
	{
		std::vector<int> ids_buffer;

		search_recursive(ids_buffer, target, distanceTol, root, 0);

		pcl::PointIndices::Ptr indices(new pcl::PointIndices());
		indices->indices = ids_buffer;

		return indices;
	}
};
