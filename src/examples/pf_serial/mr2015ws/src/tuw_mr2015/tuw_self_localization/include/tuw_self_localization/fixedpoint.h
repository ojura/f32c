#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H
#define FIXED_FRACPART 20
#define FIXED_INTPART 12

#ifndef PF_SLAVE
  #include <tuw_self_localization/config.h>
#else
  #include "config.h"
  #include <mips/io.h>
#endif 


#include <math.h>

#if defined(USEFIXED) || defined (USEFPGA)
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
    
    
    inline fixed operator -() const {
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

void inline printfixedMat( cv::Mat_<fixed> a ) {
    for(int i = 0; i < a.size().height; i++) {
        for(int j = 0; j < a.size().width; j++) 
            std::cout << double(a(i,j)) << " ";
        std::cout << std::endl;
    }
}

#endif

void inline printfixed33Array( fixed a[3][3] ) {
    printf("[ ");
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            printf("%.6lf ", double(a[i][j]));
       if(i < 2) printf("\n");
        else printf("]\n");
    }
}

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

fixed inline ftrig(fixed x, bool cos = false) {
    fixed x2, y;
    const int shift1 = 4;
    const fixed B = fixed((M_PI-3) * (1 << shift1)), C=fixed((2*M_PI-5) * (1 << shift1)), D = fixed(M_PI * (1 << shift1));
    int qN = 20;
    
    const fixed INV_PI = fixed((1<<(shift1+1))/M_PI  );
    
    fixed fx1 = x*INV_PI;
    int x1 = fx1.val;
    
    
    x1= x1<<(30-qN-shift1);          // shift to full s32 range
    
    if(cos) x1 += 1<<30;
    
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

#if defined(PF_SLAVE) && defined(HW_FSIN) 
fixed inline fsin(fixed x) {   
    fixed r;
    OUTW(IO_FSIN, x.val);
    INW(r.val, IO_FSIN);  
    return r;
}

fixed inline fcos(fixed x) {
    fixed r;
    OUTW(IO_FSIN+4, x.val);
    INW(r.val, IO_FSIN+4);  
    return r;
}

#else
fixed inline fsin(fixed x) {
    //return fixed(sin(double(x)));
    return ftrig(x, false);
}

fixed inline fcos(fixed x) {
    //return fixed(cos(double(x)));
    return ftrig(x, true);
}
#endif

    


struct lfsr {
    // 31 bit LFSR
    int state;
    
    lfsr() { };
    
    lfsr(int seed) : state(seed & ~(1<<31)) { };
    
    void update() {
        int bit = ~((state >> 0) ^ (state >> 3)) & 1;
        state = ((state >> 1) & ~(1<<30))  | (bit << 30); }
        
    fixed generate() {
        // generate a random number in [0, 1]
        for (int j = 0; j < 31; j++)
            update();        
        return fixed((state & ~(1<<(FIXED_INTPART+FIXED_FRACPART-1))) >> (FIXED_INTPART-1), true);
    }
    int generate31() {
      int oldstate = state;
        for (int j = 0; j < 31; j++)
            update();        
        return oldstate;
    }
};

struct gaussian_random{
  
    #if defined(PF_SLAVE) && defined(HW_GAUSS)
    gaussian_random() {      
      // reset lfsr init counter
      OUTW(IO_GAUSS, 0x0);
      // init LFSRs
      OUTW(IO_GAUSS+(1<<2), 0x2996EF15);
      OUTW(IO_GAUSS+(1<<2), 0x70630F8F);
      OUTW(IO_GAUSS+(1<<2), 0x59C2ED18);
      OUTW(IO_GAUSS+(1<<2), 0x291945);
      
      
      // Sanity test
      if( generate(fixed(0), fixed(1)).val != (signed int) 0xfffac301 ) {
          printf("HW Gauss is not working properly!\n");
          while(true) { };
      } 
    }
    
    inline fixed generate(fixed mi, fixed sigma) {
      fixed r;
      OUTW(IO_GAUSS+(2<<2), mi.val);
      OUTW(IO_GAUSS+(3<<2), sigma.val);    
      INW(r.val, IO_GAUSS);

      return r;
    }
    
    #else
  
    lfsr lfsrs[4];
  
    gaussian_random() {      
      // software implementation
      lfsrs[0] = 0x2996EF15;
      lfsrs[1] = 0x70630F8F;
      lfsrs[2] = 0x59C2ED18;
      lfsrs[3] = 0x291945; 
       
    }
    
    inline fixed generate(fixed mi, fixed sigma) {    
      int rands[4];
      int sum = 0;
      for(int i = 0; i<4; i++) {
          rands[i] = lfsrs[i].generate31();
          // extend sign
          rands[i] <<= 2; rands[i] >>= 2;
          
          sum += rands[i];
      }
      
      fixed r;
      r.val = sum;
      
      r = r * ((fixed(sigma << 2) * fixed( 1/(6.1993e+08 / (1<<FIXED_FRACPART)) * (1<<2))) >> 4) + mi;

      return r;
    }
    
    #endif
    
};



#endif