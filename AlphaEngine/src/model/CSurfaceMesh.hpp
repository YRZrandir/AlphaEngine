#pragma once
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

using Kernel = CGAL::Simple_cartesian<float>;
using Mesh = CGAL::Surface_mesh<Kernel::Point_3>;
