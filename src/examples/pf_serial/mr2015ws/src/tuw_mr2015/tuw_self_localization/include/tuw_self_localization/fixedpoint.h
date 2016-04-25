#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H
#define FIXED_FRACPART 20
#define FIXED_INTPART 12
#include <tuw_self_localization/config.h>
#include <math.h>

#ifdef USEFIXED
#include <opencv2/opencv.hpp>
#endif

struct fixed;
fixed inline operator*(const fixed &a, const fixed &b);
fixed inline operator>>(const fixed &a, const unsigned int &b);
fixed inline operator<<(const fixed &a, const unsigned int &b);
fixed inline operator+(const fixed &a, const fixed &b);
fixed inline operator-(const fixed &a, const fixed &b);
bool inline operator<(const fixed &a, const fixed &b);
bool inline operator>(const fixed &a, const fixed &b);

struct fixed {
    int val;
    
    fixed() : val(0) { };
    explicit inline fixed(double a) : val( a*(1<<FIXED_FRACPART) ) { };
    explicit inline fixed(int a, bool direct = false) {
        if(direct) val = a;
        else val = a << FIXED_FRACPART;
    }
    explicit inline fixed(unsigned int a, bool direct = false) {
        if(direct) val = a;
        else val = a << FIXED_FRACPART;
    }
    
    explicit inline operator double() {
        return double(this->val) / (1 << FIXED_FRACPART);
    }
    explicit inline operator int() {
        //FIXME add rounding instead of truncating?
        return this->val >> FIXED_FRACPART;
    }    
    
    
    explicit inline operator float() {
        return float(this->val) / (1 << FIXED_FRACPART);      
    }
    
    inline fixed inv() {
        // inverse using integer division
        return fixed(  (1 << (FIXED_INTPART + FIXED_FRACPART - 2)) / (val >> (FIXED_FRACPART - FIXED_INTPART + 2)), true);
    }
    
    
    inline fixed operator -() {
        fixed r; r.val = -val; return r; 
    }
    
    
};

struct fixed33mat {
    fixed mat[3][3];
};

// multiply two fixed
fixed inline operator*(const fixed &a, const fixed &b) {
    fixed r; r.val = ( (long long) a.val* (long long) b.val) >> FIXED_FRACPART; return r;
}

fixed inline operator>>(const fixed &a, const unsigned int &b) {
    fixed r; r.val =  a.val >> b; return r;
}

fixed inline operator<<(const fixed &a, const unsigned int &b) {
    fixed r; r.val =  a.val << b; return r;
}

inline fixed& operator+=(fixed &a, const fixed &b) {
    a.val += b.val;
    return a;
}

fixed inline operator+(const fixed &a, const fixed &b) {
    fixed r; r.val = a.val + b.val; return r;
}

fixed inline operator-(const fixed &a, const fixed &b) {
    fixed r; r.val = a.val - b.val; return r;
}

bool inline operator<(const fixed &a, const fixed &b) {
    return a.val < b.val;
}

bool inline operator>(const fixed &a, const fixed &b) {
    return a.val > b.val;
}



#ifdef USEFIXED
cv::Mat_<fixed> inline operator*(cv::Mat_<fixed> a, cv::Mat_<fixed> b) {
    int r1 = a.size().height, c1 = a.size().width, r2 = b.size().height, c2 = b.size().width;
    
    if(c1 != r2) { std::cout << "Wrong dimensions" << std::endl; std::exit(1); };
    cv::Mat_<fixed> r(r1, c2);
    
    for(int i=0; i<r1; ++i)
        for(int j=0; j<c2; ++j)
            r(i,j) = fixed(0);
        
        for(int i=0; i<r1; ++i)
            for(int j=0; j<c2; ++j)
                for(int k=0; k<c1; ++k)
                    r(i,j) = r(i,j) + a(i,k)*b(k,j);
                
                return r;
}

cv::Mat_<fixed> inline toFixedMat(const cv::Matx<double,3,3> &a) {
    cv::Mat_<fixed> r(3,3);
    
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++) 
            r(i,j) = fixed(a(i,j));
        
        return r;
}

void inline printMat( cv::Mat_<fixed> a ) {
    for(int i = 0; i < a.size().height; i++) {
        for(int j = 0; j < a.size().width; j++) 
            std::cout << double(a(i,j)) << " ";
        std::cout << std::endl;
    }
}
#endif

#if defined(USEFIXED) || defined(USEFPGA)
void inline toFixed33Arr(const cv::Matx<double,3,3> &a, fixed r[3][3]) {
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            r[i][j] = fixed(a(i,j));
}
#endif

/// A sine approximation via a fourth-order cosine approx. 
/// @param x   angle (with 2^15 units/circle) 
/// @return     Sine value (Q12) 
fixed inline isin_S4(fixed x) {
    int c, y;     
    static const int qN= 13, qA= 12, B=19900, C=3516;
    
    const fixed INV_PI = fixed(1/M_PI);
    
    int x1 = (x*INV_PI).val >> 6;
    
    c= x1<<(30-qN);  // Semi-circle info into carry.     
    x1 -= 1<<qN;     // sine -> cosine calc     
    
    x1= x1<<(31-qN);     // Mask with PI     
    x1= x1>>(31-qN);     // Note: SIGNED shift! (to qN)
    x1= x1*x1>>(2*qN-14); // x=x^2 To Q14    
    
    y= B - (x1*C>>14);    // B - x^2*C     
    y= (1<<qA)-(x1*y>>16); // A - x^2*(B-x^2*C)
    fixed r; r.val = (c>=0 ? y : -y) << 8; 
    
    return r; 
}

fixed inline fsin(fixed x) {
    fixed x2, y;
    const int shift1 = 4;
    const fixed B= fixed(M_PI-3) << shift1, C=fixed(2*M_PI-5) << shift1, D = fixed(M_PI) << shift1;
    int qN = 20;
    
    const fixed INV_PI = fixed((1<<(shift1+1))/M_PI  );
    
    fixed fx1 = x*INV_PI;
    int x1 = fx1.val;
    
    
    x1= x1<<(30-qN-shift1);          // shift to full s32 range
    
    if( (x1^(x1<<1)) < 0)     // test for quadrant 1 or 2
        x1= (1<<31) - x1;
    
    x1= x1>>(30-qN-shift1);
    x.val = x1;
    
    x2 = x*x;
    x2 = x2 >> shift1;
    y= C  - ((x2*B)>>shift1);
    y= D - ((x2*y)>>shift1);
    y= x * y ;
    
    return y >> (2*shift1+1);
}

fixed inline fcos(fixed x) {
    fixed x2, y;
    const int shift1 = 4;
    const fixed B= fixed(M_PI-3) << shift1, C=fixed(2*M_PI-5) << shift1, D = fixed(M_PI) << shift1;
    int qN = 20;
    
    const fixed INV_PI = fixed((1<<(shift1+1))/M_PI  );
    
    fixed fx1 = x*INV_PI;
    int x1 = fx1.val;
    
    
    x1= x1<<(30-qN-shift1);          // shift to full s32 range
    
    
    x1 += 1<<30;
    
    
    if( (x1^(x1<<1)) < 0)     // test for quadrant 1 or 2
        x1= (1<<31) - x1;
    
    x1= x1>>(30-qN-shift1);
    x.val = x1;
    
    x2 = x*x; // x=x^2 To Q14
    x2 = x2 >> shift1;
    y= C  - ((x2*B)>>shift1);
    y= D - ((x2*y)>>shift1);
    y= x * y ;
    
    return y >> (2*shift1+1);
}

struct lfsr {
    int state;
    
    lfsr() { };
    
    lfsr(int seed) : state(seed) { };
    
    void update() {
        int bit = ~((state >> 0) ^ (state >> 3)) & 1;
        state = ((state >> 1) & ~(1<<31))  | (bit << 31); }
        
        fixed generate() {
            // generate a random number in [0, 1]
            for (int j = 0; j < 32; j++)
                update();        
            return fixed(state, true) >> FIXED_INTPART;
        }
        int generate32() {
            for (int j = 0; j < 32; j++)
                update();        
            return state;
        }
};

struct gaussian_random{
    lfsr lfsrs[4];
    
    gaussian_random() {
        lfsrs[0] = 697757461;
        lfsrs[1] = 1885540239;
        lfsrs[2] = 1505946904;
        lfsrs[3] = 2693445;
    }
    
    fixed generate(fixed mi, fixed sigma) {
        int rands[4];
        int sum = 0;
        for(int i = 0; i<4; i++) {
            for (int j = 0; j < 32; j++)
                lfsrs[i].update();
            rands[i] = lfsrs[i].state;
            // extend sign
            rands[i] <<= 2; rands[i] >>= 2;
            
            sum += rands[i];
        }
        
        fixed r;
        r.val = sum;
        //std::cout << r << endl;
        
        //return r;
        
        return r * ((fixed(sigma << 2) * fixed( 1/(6.1993e+08 / (1<<FIXED_FRACPART)) * (1<<2))) >> 4) + mi; //fixed(0.001502508851200);
        
    }
    
};



#endif