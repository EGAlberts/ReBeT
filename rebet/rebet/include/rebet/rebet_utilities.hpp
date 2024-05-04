#ifndef rebet__REBET_UTILS_HPP_
#define rebet__REBET_UTILS_HPP_

#include <exception>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <queue>
#include <deque>


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


inline constexpr int ADAP_SERVICE_TIMEOUT_MILLISECOND = 2000;
inline constexpr int YOLO_SERVICE_TIMEOUT_MILLISECOND = 20000;

//https://stackoverflow.com/questions/56334492/c-create-fixed-size-queue
template <typename T, int MaxLen, typename Container=std::deque<T>>
class FixedQueue : public std::queue<T, Container> {
public:

    const Container& getContainer() const {
        return this->c;
    }
    void push(const T& value) {
        if (this->size() == MaxLen) {
           this->c.pop_front();
        }
        std::queue<T, Container>::push(value);
    }
};

#endif  // rebet__REBET_UTILS_HPP_