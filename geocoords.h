class GeoCoordinate
{
public:
    double latitude;
    double longitude;

    GeoCoordinate(double _latitude, double _longitude) {latitude = _latitude * PI / 180; longitude = _longitude * PI / 180; }
};

double getBearing(GeoCoordinate startCoord, GeoCoordinate endCoord);
double getDistance(GeoCoordinate startCoord, GeoCoordinate endCoord);