

#ifndef _TSPLINE_TSPLINE_CREATOR_H_
#define _TSPLINE_TSPLINE_CREATOR_H_

// This file defines helper functions for creation certain T-splines
// It implements planes, boxes and cylinders

#include "Tspline.h"

namespace tspline{

class TsplineCreator
{
public:
//  static void CreateUniformPlaneXY(tspline::Tspline &tsp,
//                                  double x0, double y0, double z0,
//                                  double width, double height,
//                                  unsigned segX, unsigned segY);

  static void CreateClampedPlaneXY(tspline::Tspline &tsp,
                                   double x0, double y0, double z0,
                                   double width, double height,
                                   unsigned segX, unsigned segY);

  static void CreatePlaneXY(tspline::Tspline &tsp,
                            double x0, double y0, double z0,
                            double width, double height,
                            unsigned segX, unsigned segY);

  static void CreatePlaneYZ(tspline::Tspline &tsp,
                            double x0, double y0, double z0,
                            double width, double height,
                            unsigned segY, unsigned segZ);

  static void CreatePlaneXZ(tspline::Tspline &tsp,
                            double x0, double y0, double z0,
                            double width, double height,
                            unsigned segX, unsigned segZ);

  static void CreatePlaneXY(tspline::Tspline &tsp,
                            std::vector<double> paramS,
                            std::vector<double> paramT,
                            std::vector<Point4d> controlpoints = std::vector<Point4d>());




};


}

#endif
