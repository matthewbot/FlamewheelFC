#include "matrix.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    Matrix<float, 3, 1> a = {2, 4, -8};
    Matrix<float, 3, 1> b = {-3, 9, 1};

    Matrix<float, 3, 3> out = a*tr(b);

    for (int i=0; i<out.rows(); i++) {
        for (int j=0; j<out.cols(); j++)
            cout << out(i, j) << ' ';
        cout << endl;
    }
}

