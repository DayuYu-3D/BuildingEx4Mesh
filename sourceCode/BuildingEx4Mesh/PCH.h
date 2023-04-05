#ifndef PCH_H
#define PCH_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Cartesian.h>

#include <CGAL/Timer.h>
#include <CGAL/Random.h>

#include <CGAL/Line_3.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/polygon_mesh_processing.h>
#include <CGAL/property_map.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Nef_polyhedron_3.h>

#include <CGAL/centroid.h>
#include <CGAL/barycenter.h>

#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/IO/PLY_reader.h>
#include <CGAL/IO/PLY_writer.h>
#include <CGAL/IO/write_xyz_points.h>

#include <CGAL/mesh_segmentation.h>
#include <CGAL/linear_least_squares_fitting_2.h>
#include <CGAL/linear_least_squares_fitting_3.h>

//#include <CGAL/remove_outliers.h>
//#include <CGAL/grid_simplify_point_set.h>
//#include <CGAL/jet_smooth_point_set.h>
//#include <CGAL/jet_estimate_normals.h>
//#include <CGAL/mst_orient_normals.h>
//#include <CGAL/poisson_surface_reconstruction.h>
//#include <CGAL/Advancing_front_surface_reconstruction.h>
//#include <CGAL/Scale_space_surface_reconstruction_3.h>
//#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
//#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
//#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

// Type declarations.
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;

#endif // PCH_H