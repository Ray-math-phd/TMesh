

#include "TsplineCreator.h"

using namespace tspline;

void TsplineCreator::CreateClampedPlaneXY(tspline::Tspline &tsp,
                                          double x0, double y0, double z0,
                                          double width, double height,
                                          unsigned segX, unsigned segY)
{
  double dsegX(width / segX);
  double dsegY(height / segY);
  double dgridX(width/(segX-2));
  double dgridY(height/(segY-2));

  // init vertices
  int vid(0);
  double x1, y1;
  for(unsigned j=0; j<segY+1; j++)
  {
    double ycp = y0+j*dsegY;
    double y = y0 + j * dgridY;
    for(unsigned i=0; i<segX+1; i++)
    {
      double xcp = x0+i*dsegX;
      double x = x0 + i * dgridX;
      Tspline::Vertex_handle v = CGAL::insert_point(tsp, Point2d( x, y ));
      v->data().id = vid++;
      v->data().SetCP(Point3d(xcp,ycp,z0));
      if(j==segY && i==segX)
        x1 = x;
    }
    if(j==segY)
      y1 = y;
  }

  // init grid
  for(unsigned i=0; i<segX+1; i++)
  {
    double x = x0 + i * dgridX;
    CGAL::insert( tsp, Segment2(Point2d (x, y0), Point2d (x, y1)) );
  }
  for(unsigned j=0; j<segY+1; j++)
  {
    double y = y0 + j * dgridY;
    CGAL::insert( tsp, Segment2(Point2d (x0, y), Point2d (x1, y)) );
  }

  // init edge distances
  tspline::Tspline::Halfedge_iterator eit;
  for (eit = tsp.halfedges_begin(); eit != tsp.halfedges_end(); eit++)
  {
    tspline::Tspline::Vertex_iterator v1 = eit->source();
    tspline::Tspline::Vertex_iterator v2 = eit->target();
    const Point2d& p1 = v1->point();
    const Point2d& p2 = v2->point();

    if(equal(p1.y(),p2.y())) // horizontal
    {
      if(tsp.get_left_halfedge(v1)==tsp.halfedges_end() || // left border
         tsp.get_left_halfedge(v2)==tsp.halfedges_end() ||
         tsp.get_right_halfedge(v1)==tsp.halfedges_end()||  // right border
         tsp.get_right_halfedge(v2)==tsp.halfedges_end())
        eit->data().d = 0.0;
      else
        eit->data().d = dgridX;
    }
    else if(equal(p1.x(),p2.x())) // vertical
    {
      if(tsp.get_bottom_halfedge(v1)==tsp.halfedges_end() || // bottom border
         tsp.get_bottom_halfedge(v2)==tsp.halfedges_end() ||
         tsp.get_top_halfedge(v1)==tsp.halfedges_end() ||    // top border
         tsp.get_top_halfedge(v2)==tsp.halfedges_end())
        eit->data().d = 0.0;
      else
        eit->data().d = dgridY;
    }
    else
      throw std::runtime_error("[TsplineCreator::CreatePlaneXY] Error, invalid edge.");
  }

  // update parameter space of tsp
  tsp.update_params();
  tsp.update_knot_vectors();
  tsp.clamped = false;  // this flag indicates artificial clamping
}

void TsplineCreator::CreatePlaneXY(tspline::Tspline &tsp,
                                   double x0, double y0, double z0,
                                   double width, double height,
                                   unsigned segX, unsigned segY)
{
  double dsegX(width / segX);
  double dsegY(height / segY);
  double x1,y1;
  // init vertices
  int vid(0);
  for(unsigned j=0; j<segY+1; j++)
  {
    double y = y0+j*dsegY;
    for(unsigned i=0; i<segX+1; i++)
    {
      double x = x0+i*dsegX;
      Tspline::Vertex_handle v = CGAL::insert_point(tsp, Point2d( x, y ));
      v->data().id = vid++;
      v->data().SetCP(Point3d(x,y,z0));
      if(j==segY && i==segX)
        x1 = x;
    }
    if(j==segY)
      y1 = y;
  }
  // init grid
  for(unsigned i=0; i<segX+1; i++)
  {
    double x = x0 + i * dsegX;
    CGAL::insert( tsp, Segment2(Point2d (x, y0), Point2d (x, y1)) );
  }
  for(unsigned j=0; j<segY+1; j++)
  {
    double y = y0 + j * dsegY;
    CGAL::insert( tsp, Segment2(Point2d (x0, y), Point2d (x1, y)) );
  }
  // init edge distances
  tspline::Tspline::Halfedge_iterator eit;
  for (eit = tsp.halfedges_begin(); eit != tsp.halfedges_end(); eit++)
  {
    const Point2d& p1 = eit->source()->point();
    const Point2d& p2 = eit->target()->point();
    eit->data().d = std::sqrt((p1 - p2).squared_length());
  }
  // update parameter space of tsp
  tsp.update_params();
  tsp.update_knot_vectors();
}

void TsplineCreator::CreatePlaneYZ(tspline::Tspline &tsp,
                                   double x0, double y0, double z0,
                                   double width, double height,
                                   unsigned segY, unsigned segZ)
{
  double dsegY(width / segY);
  double dsegZ(height / segZ);
  double y1,z1;

  // init vertices
  int vid(0);
  for(unsigned j=0; j<segZ+1; j++)
  {
    double z = z0+j*dsegZ;
    for(unsigned i=0; i<segY+1; i++)
    {
      double y = y0+i*dsegY;
      Tspline::Vertex_handle v = CGAL::insert_point(tsp, Point2d( y, z ));
      v->data().id = vid++;
      v->data().SetCP(Point3d(x0, y, z));
      if(j==segZ && i==segY)
        y1 = y;
    }
    if(j==segZ)
      z1 = z;
  }

  // init grid
  for(unsigned i=0; i<segY+1; i++)
  {
    double y = y0 + i * dsegY;
    CGAL::insert( tsp, Segment2(Point2d (y, z0), Point2d (y, z1)) );
  }
  for(unsigned j=0; j<segZ+1; j++)
  {
    double z = z0 + j * dsegZ;
    CGAL::insert( tsp, Segment2(Point2d (y0, z), Point2d (y1, z)) );
  }

  // init edge distances
  tspline::Tspline::Halfedge_iterator eit;
  for (eit = tsp.halfedges_begin(); eit != tsp.halfedges_end(); eit++)
  {
    const Point2d& p1 = eit->source()->point();
    const Point2d& p2 = eit->target()->point();
    eit->data().d = std::sqrt((p1 - p2).squared_length());
  }

  // update parameter space of tsp
  tsp.update_params();
  tsp.update_knot_vectors();
}

void TsplineCreator::CreatePlaneXZ(tspline::Tspline &tsp,
                                   double x0, double y0, double z0,
                                   double width, double height,
                                   unsigned segX, unsigned segZ)
{
  double dsegX(width / segX);
  double dsegZ(height / segZ);
  double x1,z1;

  // init vertices
  int vid(0);
  for(unsigned j=0; j<segZ+1; j++)
  {
    double z = z0+j*dsegZ;
    for(unsigned i=0; i<segX+1; i++)
    {
      double x = x0+i*dsegX;
      Tspline::Vertex_handle v = CGAL::insert_point(tsp, Point2d( x, z ));
      v->data().id = vid++;
      v->data().SetCP(Point3d(x,y0,z));
      if(j==segZ && i==segX)
        x1 = x;
    }
    if(j==segZ)
      z1 =z;
  }

  // init grid
  for(unsigned i=0; i<segX+1; i++)
  {
    double x = x0 + i * dsegX;
    CGAL::insert( tsp, Segment2(Point2d (x, z0), Point2d (x, z1)) );
  }
  for(unsigned j=0; j<segZ+1; j++)
  {
    double z = z0 + j * dsegZ;
    CGAL::insert( tsp, Segment2(Point2d (x0, z), Point2d (x1, z)) );
  }

  // init edge distances
  tspline::Tspline::Halfedge_iterator eit;
  for (eit = tsp.halfedges_begin(); eit != tsp.halfedges_end(); eit++)
  {
    const Point2d& p1 = eit->source()->point();
    const Point2d& p2 = eit->target()->point();
    eit->data().d = std::sqrt((p1 - p2).squared_length());
  }

  // update parameter space of tsp
  tsp.update_params();
  tsp.update_knot_vectors();
}

void TsplineCreator::CreatePlaneXY(tspline::Tspline &tsp,
                                   std::vector<double> paramS,
                                   std::vector<double> paramT,
                                   std::vector<Point4d> controlpoints)
{
  double x0 = paramS.front();
  double x1 = paramS.back();
  double y0 = paramT.front();
  double y1 = paramT.back();

  bool use_cps(false);
  if(controlpoints.size() == paramS.size() * paramT.size())
    use_cps = true;

  // init vertices
  int vid(0);
  for(unsigned j=0; j<paramT.size(); j++)
  {
    double y = paramT[j];
    for(unsigned i=0; i<paramS.size(); i++)
    {
      double x = paramS[i];
      Tspline::Vertex_handle v = CGAL::insert_point(tsp, Point2d( x, y ));
      v->data().id = vid++;
      if(use_cps)
        v->data().SetCP(controlpoints[j*paramS.size()+i]);
      else
        v->data().SetCP(Point3d(x,y,0));
    }
  }

  // init grid
  for(unsigned i=0; i<paramS.size(); i++)
  {
    double x = paramS[i];
    CGAL::insert( tsp, Segment2(Point2d (x, y0), Point2d (x, y1)) );
  }
  for(unsigned j=0; j<paramT.size(); j++)
  {
    double y = paramT[j];
    CGAL::insert( tsp, Segment2(Point2d (x0, y), Point2d (x1, y)) );
  }

  // init edge distances
  tspline::Tspline::Halfedge_iterator eit;
  for (eit = tsp.halfedges_begin(); eit != tsp.halfedges_end(); eit++)
  {
    const Point2d& p1 = eit->source()->point();
    const Point2d& p2 = eit->target()->point();
    eit->data().d = std::sqrt((p1 - p2).squared_length());
  }

  // update parameter space of tsp
  tsp.update_params();
  tsp.update_knot_vectors();
}


