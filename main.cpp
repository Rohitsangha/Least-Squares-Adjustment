#include "Definitions.h" 

//Rohit Sangha
int main()
{

MatrixXd Control;
MatrixXd Approx;
MatrixXd Distance;
MatrixXd Angles;

DataPoints("data.txt", Control,Approx,Distance,Angles); //Function To Read Text file is called.
cout << "The Control Points are (m) :" << endl<< Control << endl;
cout << "The Approximate Points are (m) :" << endl << Approx << endl;
cout << "The Distance Measured in m is :" << endl << Distance << endl;
cout << "The Angles Measured in DMS is :" << endl << Angles << endl;

MatrixXd AnglesDegrees = DMSToDegrees(Angles); //Function to Convert DMS to Degrees is called.
cout << "The Angles in degrees are :" << endl << AnglesDegrees << endl;

int r_n_1 = Distance.rows();
int r_n_2 = Angles.rows();
int u = Approx.rows()*(Approx.cols() - 1);

MatrixXd A1(r_n_1, u); //First Part of the A matrix, goes through distance measurements.
for (int i = 0; i < A1.rows(); i++) {
	vector<double> valuesA1 = DistanceModel(Distance, Control, Approx, i);
	for (int j = 0; j < A1.cols(); j++) {
		A1(i, j) = valuesA1.at(j);
		
	}
}

MatrixXd A2(r_n_2, u); //Second Part of the A matrix, goes through the angle measurements.
for (int i = 0; i < A2.rows(); i++) {
	vector<double> valuesA2 = AngleModel(AnglesDegrees, Control, Approx, i);
	for (int j = 0; j < A2.cols(); j++) {
		A2(i, j) = valuesA2.at(j);

	}
}

MatrixXd A(A1.rows() + A2.rows(), A1.cols()); // Concetonation of matricies A1 and A2 to produce final A matrix.
A << A1, A2; 
cout << "The Design Matrix A is :" << endl << A << endl;

MatrixXd w1(r_n_1, 1); //First Part of the w matrix, goes through distance measurements.
for (int i = 0; i < w1.rows(); i++) {
	for (int j = 0; j < w1.cols(); j++) {
		w1(i, j) = DistanceModel_w(Distance, Control, Approx, i);
	}
}

MatrixXd w2(r_n_2, 1); //Second Part of the w matrix, goes through angle measurements.
for (int i = 0; i < w2.rows(); i++) {
	for (int j = 0; j < w2.cols(); j++) {
		w2(i, j) = AngleModel_w(AnglesDegrees, Control, Approx, i);
	}
}

MatrixXd W(w1.rows() + w2.rows(), w1.cols()); // Concetonation of matricies w1 and w2 to produce final w matrix.
W << w1, w2;
cout << "The misclosure vector w is:" << endl << W << endl;

string forA = "The Design Matrix A is:";
string forW = "The misclosure vector w is (First 7 rows are in (m) last 5 rows are in (Degrees):";
string forDelta = "The delta vector is:";

vector <string> delta1;
for (int i = 0; i < Approx.rows(); i++) {
	string E = "E";
	string N = "N";
	int line = Approx(i, 0);
	string linenumber = to_string(line);
	string x = (E.append(linenumber));
	string y = (N.append(linenumber));
	x.append("m");
	y.append("m");
	delta1.push_back(x);
	delta1.push_back(y);
}


PrintMatrix("Output.txt", A, forA, W, forW, delta1, forDelta);



system("pause"); //Program Pauses Before Closing (So we can see any messages on the screen).
return 0;
}