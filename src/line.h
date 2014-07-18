#ifndef LINE_H
#define LINE_H
#include "includes.h"


class Line{
    // Line : y = a * x + b;
    // Note: a >= 1 / TOLERANCE should be considered as x = const;
public:
    Line(){
        a = b = 0;
    }

    Line(float a, float b){
        this->a = a; this->b = b;
    }
    Line(const Line & line){
        a = line.a; b = line.b;
    }

    float getY(float x){
        return a * x + b;
    }

    float pointDistance(float x, float y){
        return fabs(y - a * x - b) / sqrt(a * a + 1);
    }

    float getAngle(){
        if (fabs(a) >= 1 / TOLERANCE){
            return a > 0 ? M_PI / 2 : - M_PI / 2;
        }
        return atan(a);
    }

    float a, b;
};


#endif // LINE_H
