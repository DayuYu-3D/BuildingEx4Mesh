//
// Copyright 2018 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef GEOMETRY_POINT_CLOUD_LIN_SUPERVOXEL_H_
#define GEOMETRY_POINT_CLOUD_LIN_SUPERVOXEL_H_

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <queue>

#include "codelibrary/base/log.h"
#include "codelibrary/base/algorithm.h"
#include "codelibrary/base/array.h"
#include "codelibrary/base/macros.h"
#include "codelibrary/geometry/kernel/point_3d.h"
#include "codelibrary/geometry/point_cloud/grid_sample.h"
#include "codelibrary/statistics/kernel/median.h"
#include "codelibrary/util/set/disjoint_set.h"

namespace cl {
namespace geometry {
namespace point_cloud {

/**
 * Supervoxel computing via sparse subset selection.
 *
 * Given a point cloud, P = {p_1, ..., p_n}, the supervoxel segmentation is a
 * partition of the point cloud into supervoxels. Here, we define the
 * supervoxels is a subset of P, and try to find the optimal supervoxels that
 * can minimize the following energy function:
 *
 *                          N                       N   N
 *   E =  min   \lambda * (Sum I(||z_i||_p) - K) + Sum Sum(d_ij * z_ij)
 *       {z_ij}            i=1                     i=1 j=1
 *
 *                    N
 *   s.t. for any j, Sum(z_ij) = 1; and for any i, j, z_ij = {0,1};
 *                   i=1
 *
 * where ||.||_p denotes the l_p-norm and I(.) denotes the indicator function,
 * which is zero when its argument is zero and is one otherwise, K is the
 * expected number of supervoxels.
 *
 * Note that, the minimization problem of 'E' is NP-hard, so a local search
 * based method is performed, which is efficent and effective.
 *
 * Greater details please see the reference.
 *
 * Reference:
 *   Toward Better Boundary Preserved Supervoxel Segmentation for 3D Point
 *   Clouds.
 *
 * @param points        - the given point cloud.
 * @param neighbors     - the neighbors for each point.
 * @param n_supervoxels - the expected number of supervoxels.
 * @param metric        - distance metric for points.
 * @param supervoxels   - the indices of representation points.
 * @param labels        - the index of supervoxel for each point.
 */
template <typename Point, class Metric>
void SupervoxelSegmentation(const Array<Point> &points,
                            const Array<Array<int> > &neighbors,
                            int n_supervoxels,
                            const Metric &metric,
                            Array<int> *supervoxels,
                            Array<int> *labels) {
    assert(points.size() == neighbors.size());
    assert(n_supervoxels > 0);
    assert(supervoxels);
    assert(labels);

    if (points.empty()) {
        supervoxels->clear();
        labels->clear();
        return;
    }

    int n_points = points.size();

    // At first, each point is a supervoxel.
    DisjointSet set(n_points); //并查集，默认初始化时，n个元素，从0到n-1
    supervoxels->resize(n_points);
    for (int i = 0; i < n_points; ++i) {
        (*supervoxels)[i] = i;
    }

    // The size of supervoxel.
    Array<int> sizes(n_points, 1);

    // Queue for region growing.
    Array<int> queue(n_points);

    Array<Array<int> > adjacents = neighbors;

    // Minimization the energe function by fusion.
    int number_of_supervoxels = n_points;
    Array<bool> visited(n_points, false); //初始化时需要迭代n_points次，因此放在循环外边实例化。

    // Compute the minimum value of lambda.
    //计算方法：lambda0 = median(Min(点到邻域点的距离)，points中的点)
    Array<double> dis(n_points, DBL_MAX);
    for (int i = 0; i < n_points; ++i) {
        for (int j : adjacents[i]) {
            if (i != j) {
                dis[i] = std::min(dis[i], metric(points[i], points[j]));
            }
        }
    }
    double lambda = std::max(DBL_EPSILON, Median(dis.begin(), dis.end()));

    // ------------------------------------------------------------------
    // ---------------- Step 1: Find supervoxels. -----------------------
    for (; ; lambda *= 2.0) {
        if (supervoxels->size() <= 1) {
            break;
        }

        for (int i : *supervoxels) { //第一次迭代size等于points的size
            if (adjacents[i].empty()) { //附近点为空则跳出
                continue;
            }

            visited[i] = true; //初始化时size等于points，全为false。
            int front = 0, back = 1;
            queue[front++] = i; //初始化时size等于points，用户记录visited数组的索引
            for (int j : adjacents[i]) {
                j = set.Find(j);
                if (!visited[j]) {
                    visited[j] = true;
                    queue[back++] = j;
                }
            }

            Array<int> adjacent;
            while (front < back) { //相当于对i的j个邻近点
                int j = queue[front++];

                double loss = sizes[j] * metric(points[i], points[j]); //sizes的尺寸与points一样，都被初始化为1，就是公式中的Cj
                double improvement = lambda - loss;
                if (improvement > 0.0) {
                    set.Link(j, i); //合并j到i

                    sizes[i] += sizes[j]; //合并后Cj就是合并的超体素的个数

                    for (int k : adjacents[j]) {
                        k = set.Find(k);
                        if (!visited[k]) {
                            visited[k] = true;
                            queue[back++] = k;
                        }
                    }
                    adjacents[j].clear();
                    if (--number_of_supervoxels == n_supervoxels) {
                        break;
                    }
                } else {
                    adjacent.push_back(j);
                }
            }
            adjacents[i].swap(adjacent);

            for (int j = 0; j < back; ++j) {
                visited[queue[j]] = false;
            }
            if (number_of_supervoxels == n_supervoxels) {
                break;
            }
        }

        // Update supervoxels.
        number_of_supervoxels = 0;
        for (int i : *supervoxels) {
            if (set.Find(i) == i) {
                (*supervoxels)[number_of_supervoxels++] = i;
            }
        }
        supervoxels->resize(number_of_supervoxels);
        //LOG(INFO) << "The num of supervoxels: " << number_of_supervoxels;

        if (number_of_supervoxels == n_supervoxels) {
            break; //直至代表点数量等于超体素数量跳出迭代
        }
    }

    // ------------------------------------------------------------------
    // ---------------- Step 2: Refine the boundaries. ------------------
    labels->resize(n_points);
    for (int i = 0; i < n_points; ++i) {
        int j = set.Find(i);
        (*labels)[i] = j; // Assign the label to each point according to its supervoxel ID.
        dis[i] = metric(points[i], points[j]);
    }

    std::queue<int> q;
    Array<bool> in_q(n_points, false);

    for (int i = 0; i < n_points; ++i) {
        for (int j : neighbors[i]) {
            if ((*labels)[i] != (*labels)[j]) {
                if (!in_q[i]) {
                    q.push(i);
                    in_q[i] = true;
                }
                if (!in_q[j]) {
                    q.push(j);
                    in_q[j] = true;
                }
            }
        }
    }

    while (!q.empty()) {
        int i = q.front();//访问队列底元素
        q.pop(); //第一个元素出队，这是队列不是栈，先进先出
        in_q[i] = false;

        bool change = false;
        for (int j : neighbors[i]) {
            int a = (*labels)[i];
            int b = (*labels)[j];
            if (a == b) {
                continue;
            }
            double d = metric(points[i], points[b]);
            if (d < dis[i]) { //如果i的邻域中有和i的label不一致，且i到该label小于原label
                (*labels)[i] = b;
                dis[i] = d;
                change = true;
            }
        }

        if (change) {
            for (int j : neighbors[i]) {
                if ((*labels)[i] != (*labels)[j]) {
                    if (!in_q[j]) {
                        q.push(j);
                        in_q[j] = true;
                    }
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // ---------------- Step 3: Relabel the supervoxels. ----------------
    Array<int> map(n_points);
    for (int i = 0; i < supervoxels->size(); ++i) {
        map[(*supervoxels)[i]] = i;
    }
    for (int i = 0; i < n_points; ++i) {
        (*labels)[i] = map[(*labels)[i]];
        LOG(INFO) << (*labels)[i];
    }
}

/**
 * Similar to the previous one. Except that, the expected number of supervoxels
 * is determined by given resolution.
 */
template <typename Point, class Metric>
void SupervoxelSegmentation(const Array<Point> &points,
                            const Array<Array<int> > &neighbors,
                            double resolution,
                            const Metric &metric,
                            Array<int> *supervoxels,
                            Array<int> *labels) {
    Array<int> sampling;
    GridSample(points.begin(), points.end(), resolution, &sampling);
    SupervoxelSegmentation(points, neighbors, sampling.size(),
                           metric, supervoxels, labels);
}

} // namespace point_cloud
} // namespace geometry
} // namespace cl

#endif // GEOMETRY_POINT_CLOUD_LIN_SUPERVOXEL_H_
