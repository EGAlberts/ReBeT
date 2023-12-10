#ifndef rebet__REBET_UTILS_HPP_
#define rebet__REBET_UTILS_HPP_

#include <exception>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>



std::vector<double> quaternion_from_euler(double ai, double aj, double ak) 
{
    ai /= 2.0;
    aj /= 2.0;
    ak /= 2.0;
    double ci = cos(ai);
    double si = sin(ai);
    double cj = cos(aj);
    double sj = sin(aj);
    double ck = cos(ak);
    double sk = sin(ak);
    double cc = ci*ck;
    double cs = ci*sk;
    double sc = si*ck;
    double ss = si*sk;

    return {cj*sc - sj*cs, cj*ss + sj*cc, cj*cs - sj*sc, cj*cc + sj*ss};
}


#endif  // rebet__REBET_UTILS_HPP_