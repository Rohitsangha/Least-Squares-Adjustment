#ifndef DEFINITIONS_H //Include Guards Used against cross calling Headers
#define DEFINITIONS_H

//Needed Libraries
#include <iostream>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <iomanip>
#include <sstream>
#include <vector>

#include <Eigen\Dense> // Makes it so we are able to use the EIGEN library.

//Using the standard and Eigen namespace.
using namespace std;
using namespace Eigen;

//DECLARATIONS
//Function Uses Are Explained in the "Functions.cpp" Tab 
//MatrixXd& Approx, MatrixXd& Distance, MatrixXd& Angle
void DataPoints(string filename, MatrixXd& Control, MatrixXd& Approx, MatrixXd& Distance, MatrixXd& Angle); //Declaration for file reader function.
vector<double> DistanceModel(MatrixXd Distance, MatrixXd Control, MatrixXd Approx, int x); //Declaration for distance functional model function.
MatrixXd DMSToDegrees(MatrixXd DMSAngles); //Declaration for DMS to Degrees Function
vector<double> AngleModel(MatrixXd Angle, MatrixXd Control, MatrixXd Approx, int x); //Declaration for angle functional model function.
double DistanceModel_w(MatrixXd Distance, MatrixXd Control, MatrixXd Approx, int x); //Declaration for distance functional model function for w.
double AngleModel_w(MatrixXd Angle, MatrixXd Control, MatrixXd Approx, int x); //Declaration for distance for angle functional model for w.
void PrintMatrix(string filename, MatrixXd A, string A1, MatrixXd W, string w1, vector<string> delta, string delta1); //Declaration for file print function.
#endif