#include <stdio.h>
#include <math.h>
#define pi 3.14159265358979323846

double to_radians( double degree){
    double one_deg = pi / 180;
    return (one_deg * degree);
}

double distance(double lat1, double lon1, double lat2, double lon2) {
    lat1 = to_radians(lat1);
    lon1 = to_radians(lon1);
    lat2 = to_radians(lat2);
    lon2 = to_radians(lon2);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double ans = pow(sin(dlat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dlon/2),2);

    ans = 2 * asin(sqrt(ans));

    double world_rarius = 6371;

    ans = ans * world_rarius;

    return ans;
}

int main(){

    double lat1 = 18.741318;
    double lon1 = 98.943172;
    double lat2 = 18.741928;
    double lon2 = 98.941476;

    printf("%f Km", distance(lat1, lon1, lat2, lon2));

    return 0;
}