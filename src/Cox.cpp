
#include "Math.hpp"
#include "Cox.h"
#include <math.h>

using namespace tspline;

namespace tspline
{
  void cox (const double &xi, const unsigned &degree, const std::vector<double> &knots, std::vector<double> &N)
  {
    size_t nknots = knots.size();
    N.assign (nknots * (degree + 1), 0.0);

    for (unsigned p = 0; p < (degree+1); p++)
    { // loop from lower degree to higher -> unwrapped recursion

      for (unsigned s = 0; s < (nknots-1); s++)
      { // evaluate the basis N for each knotspan s

        if (p == 0)
        {
          // Equation (2.1) in [1]
          if(gequal(xi, knots[s]))
            if(smaller(xi, knots[s+1]))
              N[s] = 1.0;

          // check for interpolating knot on the right side [hacky]
          if(equal(xi,knots[nknots-1]))
          {
            if(s+1+degree < nknots)
              if(equal(knots[s+1], knots[s+1+degree]))
                N[s] = 1.0;

            if(s+degree < nknots)
              if(equal(knots[s+1], knots[s+degree]))
                N[s] = 1.0;
          }

        }
        else
        {
          // Equation (2.2)
          double A(0.0);
          double B(0.0);

          if( !equal(knots[s],knots[s+p]) )
            A = (xi - knots[s]) / (knots[s+p] - knots[s]);

          if( !equal(knots[s+1],knots[s+p+1]) )
            B = (knots[s+p+1] - xi) / (knots[s+p+1] - knots[s+1]);

          if(A < 0.0)
            A = 0.0;
          if(B < 0.0)
            B = 0.0;

          N[s + p*nknots] = A * N[s + (p-1)*nknots] + B * N[(s+1) + (p-1)*nknots];

        }
      }
    }
  }

  void coxder (const unsigned &degree, const std::vector<double> &knots, const std::vector<double> &N,
               std::vector<double> &Nd)
  {
    size_t nknots = knots.size();
    Nd.assign (nknots * (degree + 1), 0.0);
    unsigned p = degree;

    for (unsigned s = 0; s < nknots - 1; s++)
    {
      // Equation (2.12)
      double A(0.0);
      double B(0.0);

      if(!equal (knots[s+p], knots[s]))
        A = p / (knots[s+p] - knots[s]);

      if(!equal (knots[s+p+1], knots[s+1]))
        B = p / (knots[s+p+1] - knots[s+1]);

      if (A < 0.0)
        A = 0.0;
      if (B < 0.0)
        B = 0.0;

      Nd[s + p*nknots] = A * N[s + (p-1)*nknots] - B * N[(s+1) + (p-1)*nknots];

    }
  }

}
