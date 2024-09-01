#pragma once

#include <vector>
#include <numeric>
#include <algorithm>
#include <exception>
#include <functional>
#include <array>
#include <cmath>

/** @brief k-d tree class.
*/
template <class PointT>
class KDTree
{
public:
	/** @brief The constructors.
	*/
	KDTree() : root_(nullptr){};
	KDTree(const std::vector<PointT> &points) : root_(nullptr) { build(points); }

	/** @brief The destructor.
	*/
	~KDTree() { clear(); }

	/** @brief Re-builds k-d tree.
	*/
	void build(const std::vector<PointT> &points)
	{
		clear();

		points_ = points;

		std::vector<int> indices(points.size());
		std::iota(std::begin(indices), std::end(indices), 0);

		root_ = buildRecursive(indices.data(), (int)points.size(), 0);
	}

	/** @brief Clears k-d tree.
	*/
	void clear()
	{
		clearRecursive(root_);
		root_ = nullptr;
		points_.clear();
	}

	/** @brief Validates k-d tree.
	*/
	bool validate() const
	{
		try
		{
			validateRecursive(root_, 0);
		}
		catch (const Exception &)
		{
			return false;
		}

		return true;
	}

	/** @brief Searches the nearest neighbor.
	*/
	int nnSearch(const PointT &query, double *minDist = nullptr) const
	{
		int guess;
		double _minDist = std::numeric_limits<double>::max();

		nnSearchRecursive(query, root_, &guess, &_minDist);

		if (minDist)
			*minDist = _minDist;

		return guess;
	}

	/** @brief Searches k-nearest neighbors.
	*/
	std::vector<int> knnSearch(const PointT &query, int k) const
	{
		KnnQueue queue(k);
		knnSearchRecursive(query, root_, queue, k);

		std::vector<int> indices(queue.size());
		for (size_t i = 0; i < queue.size(); i++)
			indices[i] = queue[i].second;

		return indices;
	}

	/** @brief Searches neighbors within radius.
	*/
	std::vector<int> radiusSearch(const PointT &query, double radius) const
	{
		std::vector<int> indices;
		radiusSearchRecursive(query, root_, indices, radius);
		return indices;
	}

	int verticesSearch(const PointT &query, double *minDist = nullptr) const
	{
		int guess;
		double _minDist = std::numeric_limits<double>::max();

		verticesSearchRecursive(query, root_, &guess, &_minDist);

		if (minDist)
			*minDist = _minDist;

		return guess;
	}

private:
	/** @brief k-d tree node.
	*/
	struct Node
	{
		int idx;	   //!< index to the original point
		Node *next[2]; //!< pointers to the child nodes
		int axis;	   //!< dimension's axis

		Node() : idx(-1), axis(-1) { next[0] = next[1] = nullptr; }
	};

	/** @brief k-d tree exception.
	*/
	class Exception : public std::exception
	{
		using std::exception::exception;
	};

	/** @brief Bounded priority queue.
	*/
	template <class T, class Compare = std::less<T>>
	class BoundedPriorityQueue
	{
	public:
		BoundedPriorityQueue() = delete;
		BoundedPriorityQueue(size_t bound) : bound_(bound) { elements_.reserve(bound + 1); };

		void push(const T &val)
		{
			auto it = std::find_if(std::begin(elements_), std::end(elements_),
								   [&](const T &element) { return Compare()(val, element); });
			elements_.insert(it, val);

			if (elements_.size() > bound_)
				elements_.resize(bound_);
		}

		const T &back() const { return elements_.back(); };
		const T &operator[](size_t index) const { return elements_[index]; }
		size_t size() const { return elements_.size(); }

	private:
		size_t bound_;
		std::vector<T> elements_;
	};

	/** @brief Priority queue of <distance, index> pair.
	*/
	using KnnQueue = BoundedPriorityQueue<std::pair<double, int>>;

	/** @brief Builds k-d tree recursively.
	*/
	Node *buildRecursive(int *indices, int npoints, int depth)
	{
		if (npoints <= 0)
			return nullptr;

		const int axis = depth % 2;
		const int mid = (npoints - 1) / 2;

		std::nth_element(indices, indices + mid, indices + npoints, [&](int lhs, int rhs) {
			return points_[lhs][axis] < points_[rhs][axis];
		});

		Node *node = new Node();
		node->idx = indices[mid];
		node->axis = axis;

		node->next[0] = buildRecursive(indices, mid, depth + 1);
		node->next[1] = buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

		return node;
	}

	/** @brief Clears k-d tree recursively.
	*/
	void clearRecursive(Node *node)
	{
		if (node == nullptr)
			return;

		if (node->next[0])
			clearRecursive(node->next[0]);

		if (node->next[1])
			clearRecursive(node->next[1]);

		delete node;
	}

	/** @brief Validates k-d tree recursively.
	*/
	void validateRecursive(const Node *node, int depth) const
	{
		if (node == nullptr)
			return;

		const int axis = node->axis;
		const Node *node0 = node->next[0];
		const Node *node1 = node->next[1];

		if (node0 && node1)
		{
			if (points_[node->idx][axis] < points_[node0->idx][axis])
				throw Exception();

			if (points_[node->idx][axis] > points_[node1->idx][axis])
				throw Exception();
		}

		if (node0)
			validateRecursive(node0, depth + 1);

		if (node1)
			validateRecursive(node1, depth + 1);
	}

	static double distance(const PointT &p, const PointT &q)
	{
		double dist = 0;
		for (size_t i = 0; i < 2; i++)
			dist += (p[i] - q[i]) * (p[i] - q[i]);
		return sqrt(dist);
	}

	/** @brief Searches the nearest neighbor recursively.
	*/
	void nnSearchRecursive(const PointT &query, const Node *node, int *guess, double *minDist) const
	{
		if (node == nullptr)
			return;

		const PointT &train = points_[node->idx];

		const double dist = distance(query, train);
		if (dist < *minDist)
		{
			*minDist = dist;
			*guess = node->idx;
		}

		const int axis = node->axis;
		const int dir = query[axis] < train[axis] ? 0 : 1;
		nnSearchRecursive(query, node->next[dir], guess, minDist);

		const double diff = fabs(query[axis] - train[axis]);
		if (diff < *minDist)
			nnSearchRecursive(query, node->next[!dir], guess, minDist);
	}

	/** @brief Searches k-nearest neighbors recursively.
	*/
	void knnSearchRecursive(const PointT &query, const Node *node, KnnQueue &queue, int k) const
	{
		if (node == nullptr)
			return;

		const PointT &train = points_[node->idx];

		const double dist = distance(query, train);
		queue.push(std::make_pair(dist, node->idx));

		const int axis = node->axis;
		const int dir = query[axis] < train[axis] ? 0 : 1;
		knnSearchRecursive(query, node->next[dir], queue, k);

		const double diff = fabs(query[axis] - train[axis]);
		if ((int)queue.size() < k || diff < queue.back().first)
			knnSearchRecursive(query, node->next[!dir], queue, k);
	}

	/** @brief Searches neighbors within radius.
	*/
	void radiusSearchRecursive(const PointT &query, const Node *node, std::vector<int> &indices, double radius) const
	{
		if (node == nullptr)
			return;

		const PointT &train = points_[node->idx];

		const double dist = distance(query, train);
		if (dist < radius)
			indices.push_back(node->idx);

		const int axis = node->axis;
		const int dir = query[axis] < train[axis] ? 0 : 1;
		radiusSearchRecursive(query, node->next[dir], indices, radius);

		const double diff = fabs(query[axis] - train[axis]);
		if (diff < radius)
			radiusSearchRecursive(query, node->next[!dir], indices, radius);
	}


	double SegmentToPointDistance(const Vector2d &pt0, const Vector2d &pt1, const Vector2d &pt)
	{
		double A = pt[0] - pt0[0];
		double B = pt[1] - pt0[1];
		double C = pt1[0] - pt0[0];
		double D = pt1[1] - pt0[1];

		double dot = A * C + B * D;
		double len_sq = C * C + D * D;
		double param = -1;
		if (len_sq != 0) param = dot / len_sq;

		double x, y;
		if (param < 0)
		{
			x = pt0[0];
			y = pt0[1];
		}
		else if (param > 1)
		{
			x = pt1[0];
			y = pt1[1];
		}
		else
		{
			x = pt0[0] + param * C;
			y = pt0[1] + param * D;
		}

		double dx = pt[0] - x;
		double dy = pt[1] - y;

		return std::sqrt(dx * dx + dy * dy);
	}

	void verticesSearchRecursive(const PointT &query, const Node *node, int *guess, double *minDist) const
	{
		if (node == nullptr)
			return;

		const PointT &train = points_[node->idx];

		const double dist = distance(query, train);
		if (dist < *minDist)
		{
			*minDist = dist;
			*guess = node->idx;
		}

		const int axis = node->axis;
		const int dir = query[axis] < train[axis] ? 0 : 1;
		verticesSearchRecursive(query, node->next[dir], guess, minDist);

		const double diff = fabs(query[axis] - train[axis]);
		if (diff < *minDist)
			verticesSearchRecursive(query, node->next[!dir], guess, minDist);
	}

	Node *root_;				 //!< root node
	std::vector<PointT> points_; //!< points
};