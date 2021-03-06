class Filter
{
private:
    double q; //process noise covariance
    double r; //measurement noise covariance
    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain

public:
    Filter(double _q, double _r, double _p, double _intial_value);
    void update(double measurement);
    double GetValue() {return x;}
};
