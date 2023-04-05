/********************************************
 * Class: yutil
 * Author: Dayu
 * Site: WHU
 * Date: 2020528
 * email: dayuyu@whu.edu.cn
********************************************/
#pragma once

#include <vector>
#include <string>
#include <fstream>

#include "meshpaser/triangle.h"
#include "meshpaser/vec3d.h"
#include "meshpaser/vertex.h"

namespace yutil {
	
	class CalTriangArea
	{
		//////////////////////////声明////////////////////////////////////////
		/*
			// 1. 求三角形面积;返回的为平方值。该版本为CGAL版本，应该值得信赖
			 float calAreaOFtriangle_square(Vertex& v1, Vertex& v2, Vertex& v3);
			// 1. 求三角形面积;返回的为平方值
			 float calAreaOFtriangle_square(Vec3d& a, Vec3d& b, Vec3d& c);
			// 1. 求三角形面积：返回的为平方值。该版本为CGAL版本，应该值得信赖
			 float calCGALAreaOFtriangle_square(Vec3d& a, Vec3d& b, Vec3d& c);
			// 2. 将1d vector保存到文件
			template<typename T>
			void write1DVector(T& t, std::string savePath);
		*/
		
	public:
		//求三角形面积;返回的为平方值.该版本为CGAL版本，应该值得信赖
		inline float calAreaOFtriangle_square(Vertex& v1, Vertex& v2, Vertex& v3) {
			return calCGALAreaOFtriangle_square(v1._coor, v2._coor, v3._coor);
		}

		//求三角形面积;返回的为平方值
		float calAreaOFtriangle_square(Vec3d& a, Vec3d& b, Vec3d& c) {
			float area = -1;

			float side[3];//存储三条边的长度;

			side[0] = sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2) + pow(a.z() - b.z(), 2));
			side[1] = sqrt(pow(a.x() - c.x(), 2) + pow(a.y() - c.y(), 2) + pow(a.z() - c.z(), 2));
			side[2] = sqrt(pow(c.x() - b.x(), 2) + pow(c.y() - b.y(), 2) + pow(c.z() - b.z(), 2));

			//不能构成三角形;
			if (side[0] + side[1] <= side[2] || side[0] + side[2] <= side[1] || side[1] + side[2] <= side[0]) return area;

			//利用海伦公式。s=sqr(p*(p-a)(p-b)(p-c)); 
			float p = (side[0] + side[1] + side[2]) / 2; //半周长;
			area = p * (p - side[0]) * (p - side[1]) * (p - side[2]);
			return area;//返回-1为不能组成三角形;
		}

		//求三角形面积：返回的为平方值。该版本为CGAL版本，应该值得信赖
		inline float calCGALAreaOFtriangle_square(Vec3d& a, Vec3d& b, Vec3d& c) {
			using FT = float;

			// Compute vectors pq and pr, then the cross product,
			// then 1/4 of its squared length.
			FT dqx = b.x() - a.x();
			FT dqy = b.y() - a.y();
			FT dqz = b.z() - a.z();
			FT drx = c.x() - a.x();
			FT dry = c.y() - a.y();
			FT drz = c.z() - a.z();

			FT vx = dqy * drz - dqz * dry;
			FT vy = dqz * drx - dqx * drz;
			FT vz = dqx * dry - dqy * drx;

			return (vx * vx + vy * vy + vz * vz) / 4;
		}

		template<typename T>
		static void write1DVector(T& t, std::string savePath)
		{
			std::ofstream out;
			out.open(savePath);
			for (auto& ss : t) {
				out << ss << std::endl;
			}
			out.close();
		}
	};
}