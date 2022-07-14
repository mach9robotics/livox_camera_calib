#include <Eigen/Core>
#include <iostream>
#include <math.h>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main()
{

    Eigen::Matrix3f m;

    m = AngleAxisf(1.5708, Vector3f::UnitX())
    * AngleAxisf(0.0, Vector3f::UnitY())
    * AngleAxisf(1.5708, Vector3f::UnitZ());

    cout << "original rotation:" << endl;
    cout << m << endl << endl;

    Vector3f ea = m.eulerAngles(2, 1, 0); 
    cout << "to Euler angles:" << endl;
    cout << ea << endl << endl;

    Eigen::Matrix3f n;
    n = AngleAxisf(ea[0], Vector3f::UnitZ())
    * AngleAxisf(ea[1], Vector3f::UnitY())
    * AngleAxisf(ea[2], Vector3f::UnitX()); 

    cout << "recalc original rotation:" << endl;
    cout << n << endl;

    return 0;
}

