

#ifndef _TSPLINE_COX_H_
#define _TSPLINE_COX_H_

#include <vector>

namespace tspline
{

/** @brief compute basis functions using cox-de-boor */
void cox (const double &xi, const unsigned &degree, const std::vector<double> &knots, std::vector<double> &N);

/** @brief compute derivatives of basis functions using cox-de-boor */
void coxder (const unsigned &degree, const std::vector<double> &knots, const std::vector<double> &N,
             std::vector<double> &Nd);

}

#endif
