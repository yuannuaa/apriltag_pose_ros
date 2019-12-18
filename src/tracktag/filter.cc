//This is a constant velocity extend kalman filter implementation

#include "filter.h"




//Definition

void constant_filter::init(){
    F << 1 , 0 , 0 , dt ,  0 ,  0,
         0 , 1 , 0 , 0  , dt ,  0,
         0 , 0 , 1 , 0  ,  0 , dt,
         0 , 0 , 0 , 1  ,  0 ,  0,
         0 , 0 , 0 , 0  ,  1 ,  0,
         0 , 0 , 0 , 0  ,  0 ,  1; // F constant velocity 

    P << 1 , 0 , 0 , 0  ,  0 ,  0,
         0 , 1 , 0 , 0  ,  0 ,  0,
         0 , 0 , 1 , 0  ,  0 ,  0,
         0 , 0 , 0 , 1  ,  0 ,  0,
         0 , 0 , 0 , 0  ,  1 ,  0,
         0 , 0 , 0 , 0  ,  0 ,  1;

    H << 1 , 0 , 0 , dt ,  0 ,  0,
         0 , 1 , 0 , 0  , dt ,  0,
         0 , 0 , 1 , 0  ,  0 , dt;

    R << 0.1 ,  0 ,
          0  , 0.1;

    
    
}

