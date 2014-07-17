#ifndef LINE_H
#define LINE_H



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

    float a, b;
};

class Line2{
    // Line: a * x + b * y = 1;

public:
    Line2(){
        a = b = 0;
    }

    Line2 (float a, float b){
        this->a = a;
        this->b = b;
    }

    Line2 (const Line2 & line){
        a = line.a; b = line.b;
    }

private:
    float a, b;
};


#endif // LINE_H
