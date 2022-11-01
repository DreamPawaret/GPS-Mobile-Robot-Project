#include <stdio.h>
#include <math.h>
#define PI 3.14159265358979323846
int is_turnleft = 0;

//เปลี่ยนค่าจากเรียนเดียนเป็นองศา
double to_degree(double radians){
    radians = (radians * 180) / PI;

    return radians;
}

//มุมที่รถต้องหมุนไปยังทิศจุดหมาย
double rotate_angle(double lat1, double lon1, double lat2, double lon2, double car_angle_ref_n){
    double dlat = lat2 - lat1;
    double dlon = lon2 -lon1;
    double angle_ref_x;

    angle_ref_x = atan(dlat / dlon); //result is radians
    angle_ref_x = to_degree(angle_ref_x); //result is degree

    printf("%f\n", angle_ref_x);
    
    
    is_turnleft = 0;//-----------------------------------------------------------------------------turn right
    if(dlon>0 && dlat>0){//----------------------------------------------x+ y+ 
        return 90 - (car_angle_ref_n + angle_ref_x);
    }

    if(dlon>0 && dlat==0){//----------------------------------------------x+ y=0
        return 90 - (car_angle_ref_n + angle_ref_x);
    }

    if(dlon>0 && dlat<0){//----------------------------------------------x+ y-
        return (90 + abs(angle_ref_x)) - car_angle_ref_n;
    }

    if(dlon==0 && dlat<0){//----------------------------------------------x=0 y-
        return 180 - car_angle_ref_n;
    }

    is_turnleft = 1; //----------------------------------------------------------------------------turn left
    if(dlon<0 && dlat<0){//----------------------------------------------x- y-
        return (270 - angle_ref_x) - car_angle_ref_n;
    }

    if(dlon<0 && dlat==0){//----------------------------------------------x- y=0
        return 270 - car_angle_ref_n;
    }

    if(dlon<0 && dlat>0){//----------------------------------------------x- y+
        return 270 + abs(angle_ref_x) -car_angle_ref_n;
    }

    if(dlon==0 && dlat>0){//----------------------------------------------x=0 y+
        return 360 - car_angle_ref_n;
    }
    
}


int main(){
    double lat1 = 0;
    double lon1 = 0;
    double lat2 = 100;
    double lon2 = 0;
    double car_angle_ref_n = 15;

    double rotate_ang = rotate_angle(lat1, lon1, lat2, lon2, car_angle_ref_n);

    printf("%f", rotate_ang);

    return 0;
}