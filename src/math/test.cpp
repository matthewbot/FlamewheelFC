#include "matrix.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    Vector<float, 3> a = {2, 4, -8};
    Vector<float, 3> b = {-3, 9, 1};

    cout << dot(a, b+a) << endl;

    Vector<float, 3> out = cross(a+b, b-a)/5;

    cout << out[0] << ' ' << out[1] << ' ' << out[2] << endl;

    Matrix<float, 3, 3> mat = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    mat.slice<2, 2>(0, 0) = 2 * tr(mat.slice<2, 2>(1, 1));

    for (int r=0; r<mat.rows(); r++) {
        for (int c=0; c<mat.cols(); c++) {
            cout << mat(r, c) << ' ';
        }
        cout << endl;
    }
}

