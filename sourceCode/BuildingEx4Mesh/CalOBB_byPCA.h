#pragma once
/********************************************
 * Class: CalOBB_byPCA
 * Author: Dayu
 * Site: WHU
 * Date: 2020504
 * email: dayuyu@whu.edu.cn
 * Des: 纯头文件类，方便调用
********************************************/

#include <vector>
#include <array>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> // for EigenSolver

template <typename PointT>
class CalOBB_byPCA
{
public:
    using PolintsVector = typename std::vector<PointT>;
public:
    CalOBB_byPCA(): 
        _is_valid(false),
        _mean_value(0.0f, 0.0f, 0.0f),
        _major_axis(0.0f, 0.0f, 0.0f),
        _middle_axis(0.0f, 0.0f, 0.0f),
        _minor_axis(0.0f, 0.0f, 0.0f),
        _major_value(0),
        _middle_value(0),
        _minor_value(0),
        _obb_min_point(),
        _obb_max_point(),
        _obb_position(0.0f, 0.0f, 0.0f) {}
    ~CalOBB_byPCA(){}

    /*
    * Necessary step
    */
    void setInputPoints(PolintsVector* inputPoints)
    {
        _inputPoints = inputPoints;

        _is_valid= false;
        _mean_value[0] = 0.f; _mean_value[1] = 0.f; _mean_value[2] = 0.f;
        _major_axis[0] = 0.f; _major_axis[1] = 0.f; _major_axis[2] = 0.f;
        _middle_axis[0] = 0.f; _middle_axis[1] = 0.f; _middle_axis[2] = 0.f;
        _minor_axis[0] = 0.f; _minor_axis[1] = 0.f; _minor_axis[2] = 0.f;
        _major_value = 0;
        _middle_value = 0;
        _minor_value = 0;
        _obb_min_point[0] = 0.f; _obb_min_point[1] = 0.f; _obb_min_point[2] = 0.f;
        _obb_max_point[0] = 0.f; _obb_max_point[1] = 0.f; _obb_max_point[2] = 0.f;
        _obb_position[0] = 0.f; _obb_position[1] = 0.f; _obb_position[2] = 0.f;
    }

    /*
    * 执行函数
    */
    void compute()
    {
        this->computeMeanValue(); //计算均值

        Eigen::Matrix <float, 3, 3> covariance_matrix; 
        this->computeCovarianceMatrix(covariance_matrix); //协方差矩阵

        this->computeEigenVectors(covariance_matrix, _major_axis, _middle_axis, _minor_axis,
            _major_value, _middle_value, _minor_value);

        _obb_min_point[0] = std::numeric_limits <float>::max();
        _obb_min_point[1] = std::numeric_limits <float>::max();
        _obb_min_point[2] = std::numeric_limits <float>::max();

        _obb_max_point[0] = std::numeric_limits <float>::min();
        _obb_max_point[1] = std::numeric_limits <float>::min();
        _obb_max_point[2] = std::numeric_limits <float>::min();

        size_t numPoints = _inputPoints->size();
        //if (!numPoints) {
        //    _is_valid = false;
        //    std::cout << "asfbefs warning" << std::endl;
        //    return;
        //}
        for (size_t i = 0; i < numPoints; ++i)
        {
            float x = (_inputPoints->at(i)[0] - _mean_value(0)) * _major_axis(0) +
                      (_inputPoints->at(i)[1] - _mean_value(1)) * _major_axis(1) +
                      (_inputPoints->at(i)[2] - _mean_value(2)) * _major_axis(2);
            float y = (_inputPoints->at(i)[0] - _mean_value(0)) * _middle_axis(0) +
                      (_inputPoints->at(i)[1] - _mean_value(1)) * _middle_axis(1) +
                      (_inputPoints->at(i)[2] - _mean_value(2)) * _middle_axis(2);
            float z = (_inputPoints->at(i)[0] - _mean_value(0)) * _minor_axis(0) +
                      (_inputPoints->at(i)[1] - _mean_value(1)) * _minor_axis(1) +
                      (_inputPoints->at(i)[2] - _mean_value(2)) * _minor_axis(2);

            if (x <= _obb_min_point[0]) _obb_min_point[0] = x;
            if (y <= _obb_min_point[1]) _obb_min_point[1] = y;
            if (z <= _obb_min_point[2]) _obb_min_point[2] = z;

            if (x >= _obb_max_point[0]) _obb_max_point[0] = x;
            if (y >= _obb_max_point[1]) _obb_max_point[1] = y;
            if (z >= _obb_max_point[2]) _obb_max_point[2] = z;
        }

        _obb_rotational_matrix << _major_axis(0), _middle_axis(0), _minor_axis(0),
                                  _major_axis(1), _middle_axis(1), _minor_axis(1),
                                  _major_axis(2), _middle_axis(2), _minor_axis(2);

        Eigen::Vector3f shift(
            (_obb_max_point[0] + _obb_min_point[0]) / 2.0f,
            (_obb_max_point[1] + _obb_min_point[1]) / 2.0f,
            (_obb_max_point[2] + _obb_min_point[2]) / 2.0f);

        _obb_min_point[0] -= shift(0);
        _obb_min_point[1] -= shift(1);
        _obb_min_point[2] -= shift(2);

        _obb_max_point[0] -= shift(0);
        _obb_max_point[1] -= shift(1);
        _obb_max_point[2] -= shift(2);

        _obb_position = _mean_value + _obb_rotational_matrix * shift;

        _is_valid = true;
    }

    /** \brief This method gives access to the computed oriented bounding box. It returns true
      * if the current values (eccentricity, moment of inertia etc) are valid and false otherwise.
      * Note that in order to get the OBB, each vertex of the given AABB (specified with min_point and max_point)
      * must be rotated with the given rotational matrix (rotation transform) and then positioned.
      * Also pay attention to the fact that this is not the minimal possible bounding box. This is the bounding box
      * which is oriented in accordance with the eigen vectors.
      * \param[out] min_point min point of the OBB
      * \param[out] max_point max point of the OBB
      * \param[out] position position of the OBB
      * \param[out] rotational_matrix this matrix represents the rotation transform
      */
    bool getOBB(PointT& min_point, PointT& max_point,
                PointT& position, Eigen::Matrix3f& rotational_matrix) const
    {
        min_point = _obb_min_point;
        max_point = _obb_max_point;
        position[0] = _obb_position(0);
        position[1] = _obb_position(1);
        position[2] = _obb_position(2);
        rotational_matrix = _obb_rotational_matrix;

        return _is_valid;
    }

    //OBB转为OBB框的八个顶点
    std::array<PointT, 8> ConvertOBB2Array(PointT& min_point_OBB, PointT& max_point_OBB,
        PointT& position, Eigen::Matrix3f& rotational_matrix_OBB) const
    {
        Eigen::Vector3f p1(min_point_OBB[0], min_point_OBB[1], min_point_OBB[2]);
        Eigen::Vector3f p2(min_point_OBB[0], min_point_OBB[1], max_point_OBB[2]);
        Eigen::Vector3f p3(max_point_OBB[0], min_point_OBB[1], max_point_OBB[2]);
        Eigen::Vector3f p4(max_point_OBB[0], min_point_OBB[1], min_point_OBB[2]);
        Eigen::Vector3f p5(min_point_OBB[0], max_point_OBB[1], min_point_OBB[2]);
        Eigen::Vector3f p6(min_point_OBB[0], max_point_OBB[1], max_point_OBB[2]);
        Eigen::Vector3f p7(max_point_OBB[0], max_point_OBB[1], max_point_OBB[2]);
        Eigen::Vector3f p8(max_point_OBB[0], max_point_OBB[1], min_point_OBB[2]);

        Eigen::Vector3f postion1(position[0], position[1], position[2]);

        p1 = rotational_matrix_OBB * p1 + postion1;
        p2 = rotational_matrix_OBB * p2 + postion1;
        p3 = rotational_matrix_OBB * p3 + postion1;
        p4 = rotational_matrix_OBB * p4 + postion1;
        p5 = rotational_matrix_OBB * p5 + postion1;
        p6 = rotational_matrix_OBB * p6 + postion1;
        p7 = rotational_matrix_OBB * p7 + postion1;
        p8 = rotational_matrix_OBB * p8 + postion1;       

        return { Vec3d(p1[0], p1[1], p1[2]),
                 Vec3d(p2[0], p2[1], p2[2]),
                 Vec3d(p3[0], p3[1], p3[2]),
                 Vec3d(p4[0], p4[1], p4[2]),
                 Vec3d(p5[0], p5[1], p5[2]),
                 Vec3d(p6[0], p6[1], p6[2]),
                 Vec3d(p7[0], p7[1], p7[2]),
                 Vec3d(p8[0], p8[1], p8[2]) };
    }

protected:

    //计算平均值
    void computeMeanValue()
    {
        _mean_value(0) = 0.0f;
        _mean_value(1) = 0.0f;
        _mean_value(2) = 0.0f;      

        size_t number_of_points = _inputPoints->size();
        for (size_t i = 1; i < number_of_points; ++i)
        {
            _mean_value(0) += _inputPoints->at(i)[0];
            _mean_value(1) += _inputPoints->at(i)[1];
            _mean_value(2) += _inputPoints->at(i)[2];            
        }

        if (number_of_points == 0)
            number_of_points = 1;

        _mean_value /= number_of_points;
    }

    //计算协方差矩阵
    void computeCovarianceMatrix(Eigen::Matrix <float, 3, 3>& covariance_matrix)
    {
        covariance_matrix.setZero();

        size_t number_of_points = _inputPoints->size();
        Eigen::Matrix<float, 4, 1> pt;
        for (size_t i = 0; i < number_of_points; ++i)
        {  
            const auto& p = _inputPoints->at(i);
            pt(0) = p[0] - _mean_value(0);
            pt(1) = p[1] - _mean_value(1);
            pt(2) = p[2] - _mean_value(2);

            covariance_matrix(1, 1) += pt.y() * pt.y();
            covariance_matrix(1, 2) += pt.y() * pt.z();
            covariance_matrix(2, 2) += pt.z() * pt.z();

            pt *= pt.x();
            covariance_matrix(0, 0) += pt.x();
            covariance_matrix(0, 1) += pt.y();
            covariance_matrix(0, 2) += pt.z();
        }
    }

    //计算特征向量和特征值
    void computeEigenVectors(
        const Eigen::Matrix <float, 3, 3>& covariance_matrix,
        Eigen::Vector3f& major_axis, Eigen::Vector3f& middle_axis, 
        Eigen::Vector3f& minor_axis, float& major_value,
        float& middle_value, float& minor_value)
    {
        Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> > eigen_solver;
        eigen_solver.compute(covariance_matrix);

        Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvectorsType eigen_vectors;
        Eigen::EigenSolver <Eigen::Matrix <float, 3, 3> >::EigenvalueType eigen_values;
        eigen_vectors = eigen_solver.eigenvectors();
        eigen_values = eigen_solver.eigenvalues();

        unsigned int temp = 0;
        unsigned int major_index = 0;
        unsigned int middle_index = 1;
        unsigned int minor_index = 2;

        if (eigen_values.real() (major_index) < eigen_values.real() (middle_index))
        {
            temp = major_index;
            major_index = middle_index;
            middle_index = temp;
        }

        if (eigen_values.real() (major_index) < eigen_values.real() (minor_index))
        {
            temp = major_index;
            major_index = minor_index;
            minor_index = temp;
        }

        if (eigen_values.real() (middle_index) < eigen_values.real() (minor_index))
        {
            temp = minor_index;
            minor_index = middle_index;
            middle_index = temp;
        }

        major_value = eigen_values.real() (major_index);
        middle_value = eigen_values.real() (middle_index);
        minor_value = eigen_values.real() (minor_index);

        major_axis = eigen_vectors.col(major_index).real();
        middle_axis = eigen_vectors.col(middle_index).real();
        minor_axis = eigen_vectors.col(minor_index).real();

        major_axis.normalize();
        middle_axis.normalize();
        minor_axis.normalize();

        float det = major_axis.dot(middle_axis.cross(minor_axis));
        if (det <= 0.0f)
        {
            major_axis(0) = -major_axis(0);
            major_axis(1) = -major_axis(1);
            major_axis(2) = -major_axis(2);
        }
    }

private:

    bool _is_valid;

    /** \brief 输入的点云 */
    const PolintsVector* _inputPoints;

    /** \brief Stores the mean value (center of mass) of the cloud */
    Eigen::Vector3f _mean_value;

    /** \brief 特征向量：Major eigen vector */
    Eigen::Vector3f _major_axis;

    /** \brief 特征向量：Middle eigen vector */
    Eigen::Vector3f _middle_axis;

    /** \brief 特征向量：Minor eigen vector */
    Eigen::Vector3f _minor_axis;

    /** \brief 特征值：Major eigen value */
    float _major_value;

    /** \brief 特征值：Middle eigen value */
    float _middle_value;

    /** \brief 特征值：Minor eigen value */
    float _minor_value;

    /** \brief Min point of the oriented bounding box */
    PointT _obb_min_point;

    /** \brief Max point of the oriented bounding box */
    PointT _obb_max_point;

    /** \brief Stores position of the oriented bounding box */
    Eigen::Vector3f _obb_position;

	/** \brief Stores the rotational matrix of the oriented bounding box */
	Eigen::Matrix3f _obb_rotational_matrix;
};