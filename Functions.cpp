#include "Definitions.h" 

//FUNCTIONS

//Function ONE: Read the input file into multiple eigen matrix
//MatrixXd& Control, MatrixXd& Approx, MatrixXd& Distance, MatrixXd& Angle
void DataPoints(string filename, MatrixXd& Control, MatrixXd& Approx, MatrixXd& Distance, MatrixXd& Angle) {
	ifstream file_in;
	file_in.open(filename, ifstream::in);

	if (file_in.fail()) {
		cout << "File could not be accessed";
		exit(0);
	}

	{
		int length;
		string section = "CONTROL POINT";
		string line;
		while (getline(file_in, line)) {
			if (line.find(section) != string::npos) {
				length = file_in.tellg();
			}
		}

		file_in.clear();
		file_in.seekg(length, file_in.beg);


		int rows = 0;
		int cols = 0;
		string r;
		double temp2;

		while (!file_in.eof()) {
			getline(file_in, r);
			rows++;
			if (file_in.peek() == '\n')
				break;
		}

		stringstream temp;
		temp << r;
		while (temp >> temp2) {
			cols++;
		}

		file_in.clear();
		file_in.seekg(length, file_in.beg);

		Control.resize(rows, cols);

		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				double x;
				file_in >> x;
				Control(i, j) = x;
			}
		}

		file_in.clear();
		file_in.seekg(0, file_in.beg);
	}
	
	//Next Data Set
	{
		int length;
		string section = "APPROXIMATE POINT";
		string line;
		while (getline(file_in, line)) {
			if (line.find(section) != string::npos) {
				length = file_in.tellg();
			}
		}

		file_in.clear();
		file_in.seekg(length, file_in.beg);


		int rows = 0;
		int cols = 0;
		string r;
		double temp2;

		while (!file_in.eof()) {
			getline(file_in, r);
			rows++;
			if (file_in.peek() == '\n')
				break;
		}

		stringstream temp;
		temp << r;
		while (temp >> temp2) {
			cols++;
		}

		file_in.clear();
		file_in.seekg(length, file_in.beg);

		Approx.resize(rows, cols);

		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				double x;
				file_in >> x;
				Approx(i, j) = x;
			}
		}

		file_in.clear();
		file_in.seekg(0, file_in.beg);
	}
	
	//Next Data Set
	{
		int length;
		string section = "DISTANCES";
		string line;
		while (getline(file_in, line)) {
			if (line.find(section) != string::npos) {
				length = file_in.tellg();
			}
		}

		file_in.clear();
		file_in.seekg(length, file_in.beg);


		int rows = 0;
		int cols = 0;
		string r;
		double temp2;

		while (!file_in.eof()) {
			getline(file_in, r);
			rows++;
			if (file_in.peek() == '\n')
				break;
		}

		stringstream temp;
		temp << r;
		while (temp >> temp2) {
			cols++;
		}

		file_in.clear();
		file_in.seekg(length, file_in.beg);

		Distance.resize(rows, cols);

		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				double x;
				file_in >> x;
				Distance(i, j) = x;
			}
		}

		file_in.clear();
		file_in.seekg(0, file_in.beg);
	}

	//Next Data Set
	{
		int length;
		string section = "ANGLES";
		string line;
		while (getline(file_in, line)) {
			if (line.find(section) != string::npos) {
				length = file_in.tellg();
			}
		}

		file_in.clear();
		file_in.seekg(length, file_in.beg);


		int rows = 0;
		int cols = 0;
		string r;
		double temp2;

		while (!file_in.eof()) {
			getline(file_in, r);
			rows++;
			if (file_in.peek() == '\n')
				break;
		}

		stringstream temp;
		temp << r;
		while (temp >> temp2) {
			cols++;
		}

		file_in.clear();
		file_in.seekg(length, file_in.beg);

		Angle.resize(rows, cols);

		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				double x;
				file_in >> x;
				Angle(i, j) = x;
			}
		}

		file_in.clear();
		file_in.seekg(0, file_in.beg);
	}

	file_in.close();
	
}

//Funcion to convert DMS to Degrees
MatrixXd DMSToDegrees(MatrixXd DMSAngles) {
	MatrixXd Degrees(DMSAngles.rows(), DMSAngles.cols() - 2);

		for (int i = 0; i < DMSAngles.rows();i++) {
			Degrees(i, 0) = DMSAngles(i, 0);
		}
		for (int i = 0; i < DMSAngles.rows(); i++) {
			Degrees(i, 1) = DMSAngles(i, 1) + DMSAngles(i, 2) / 60 + DMSAngles(i, 3) / 3600;
		}
		return Degrees;
		
}

//Distance Functional Model for A
vector<double> DistanceModel(MatrixXd Distance, MatrixXd Control, MatrixXd Approx, int x) {
	int line = Distance(x, 0);
	string linenumber = to_string(line);
	double EA = 0;
	double NA = 0;
	double EB = 0;
	double NB = 0;
	MatrixXi ToFrom(2, 1);
	vector <double> returnval;
	for (int i = 0; i < Approx.rows(); i++) {
		for (int j = 1; j < Approx.cols(); j++) {

			bool contains = false;
			int ApproxPoint = Approx(i, 0);
			double value = Approx(i, j);

			for (int i = 0; i < linenumber.size(); i++) {
				int x = linenumber.at(i);
				ToFrom(i, 0) = x - 48;

				if (ToFrom(i,0) == ApproxPoint) {
					contains = true;
				}
			}

			if (contains == false) {
				returnval.push_back(0);
			}

			for (int i = 0; i < Control.rows(); i++) {
				if (Control(i, 0) == ToFrom(0, 0)) {
					EA = Control(i, 1);
					NA = Control(i, 2);
				}

				if (Control(i, 0) == ToFrom(1, 0)) {
					EB = Control(i, 1);
					NB = Control(i, 2);
				}
			}

			for (int i = 0; i < Approx.rows(); i++) {
				if (Approx(i, 0) == ToFrom(0, 0)) {
					EA = Approx(i, 1);
					NA = Approx(i, 2);
				}

				if (Approx(i, 0) == ToFrom(1, 0)) {
					EB = Approx(i, 1);
					NB = Approx(i, 2);
				}
			}

			double dab = sqrt(pow(EB - EA, 2) + pow(NB - NA, 2));

			if (value == EA) {
				returnval.push_back((EB - EA) / dab);
			}
			else if (value == NA) {
				returnval.push_back((NB - NA) / dab);
			}
			else if (value == EB) {
				returnval.push_back(-(EB - EA) / dab);
			}
			else if (value == NB) {
				returnval.push_back(-(NB - NA) / dab);
			}
		}
	}
	return returnval;
}

//Angle Functional Model for A
vector<double> AngleModel(MatrixXd Angle, MatrixXd Control, MatrixXd Approx, int x) {
	int line = Angle(x, 0);
	string linenumber = to_string(line);
	double EA = 0;
	double NA = 0;
	double EB = 0;
	double NB = 0;
	double EC = 0;
	double NC = 0;
	double dcb;
	double dca;
	MatrixXi ToFromAt(3, 1);
	vector <double> returnval;
	for (int i = 0; i < Approx.rows(); i++) {
		for (int j = 1; j < Approx.cols(); j++) {

			bool contains = false;
			int ApproxPoint = Approx(i, 0);
			double value = Approx(i, j);

			for (int i = 0; i < linenumber.size(); i++) {
				int x = linenumber.at(i);
				ToFromAt(i, 0) = x - 48;

				if (ToFromAt(i, 0) == ApproxPoint) {
					contains = true;
				}
			}

			if (contains == false) {
				returnval.push_back(0);
			}

			for (int i = 0; i < Control.rows(); i++) {
				if (Control(i, 0) == ToFromAt(0, 0)) {
					EA = Control(i, 1);
					NA = Control(i, 2);
				}

				if (Control(i, 0) == ToFromAt(1, 0)) {
					EB = Control(i, 1);
					NB = Control(i, 2);
				}

				if (Control(i, 0) == ToFromAt(2, 0)) {
					EC = Control(i, 1);
					NC = Control(i, 2);
				}
			}

			for (int i = 0; i < Approx.rows(); i++) {
				if (Approx(i, 0) == ToFromAt(0, 0)) {
					EA = Approx(i, 1);
					NA = Approx(i, 2);
				}

				if (Approx(i, 0) == ToFromAt(1, 0)) {
					EB = Approx(i, 1);
					NB = Approx(i, 2);
				}

				if (Approx(i, 0) == ToFromAt(2, 0)) {
					EC = Approx(i, 1);
					NC = Approx(i, 2);
				}
			}

			dca = sqrt(pow(EA - EC, 2) + pow(NA - NC, 2));
			dcb = sqrt(pow(EB - EC, 2) + pow(NB - NC, 2));

			if (value == EA) {
				returnval.push_back((NA - NC) / (pow(dca,2)));
			}
			else if (value == NA) {
				returnval.push_back(-(EA - EC) / (pow(dca, 2)));
			}
			else if (value == EB) {
				returnval.push_back(-(NB - NC) / (pow(dcb, 2)));
			}
			else if (value == NB) {
				returnval.push_back((EB - EC) / (pow(dcb, 2)));


			}
		}
	}
	return returnval;
}

//Distance Functional Model for w
double DistanceModel_w(MatrixXd Distance, MatrixXd Control, MatrixXd Approx, int x) {
	int line = Distance(x, 0);
	double value = Distance(x, 1);
	string linenumber = to_string(line);
	double EA = 0;
	double NA = 0;
	double EB = 0;
	double NB = 0;
	MatrixXi ToFrom(2, 1);

	for (int i = 0; i < linenumber.size(); i++) {
		int x = linenumber.at(i);
		ToFrom(i, 0) = x - 48;
	}

	for (int i = 0; i < Control.rows(); i++) {
		if (Control(i, 0) == ToFrom(0, 0)) {
		EA = Control(i, 1);
		NA = Control(i, 2);
		}
		if (Control(i, 0) == ToFrom(1, 0)) {
		EB = Control(i, 1);
		NB = Control(i, 2);
		}
	}
	for (int i = 0; i < Approx.rows(); i++) {
		if (Approx(i, 0) == ToFrom(0, 0)) {
		EA = Approx(i, 1);
		NA = Approx(i, 2);
		}

		if (Approx(i, 0) == ToFrom(1, 0)) {
		EB = Approx(i, 1);
		NB = Approx(i, 2);
		}
	}

double dab = sqrt(pow(EB - EA, 2) + pow(NB - NA, 2));
double w = value - dab;
return w;
}

//Angle Model for w
double AngleModel_w(MatrixXd Angle, MatrixXd Control, MatrixXd Approx, int x) {
	int line = Angle(x, 0);
	double value = Angle(x, 1);
	string linenumber = to_string(line);
	double EA = 0;
	double NA = 0;
	double EB = 0;
	double NB = 0;
	double EC = 0;
	double NC = 0;
	double PI = 3.14159265358979323846;
	MatrixXi ToFromAt(3, 1);

	for (int i = 0; i < linenumber.size(); i++) {
		int x = linenumber.at(i);
		ToFromAt(i, 0) = x - 48;
	}

	for (int i = 0; i < Control.rows(); i++) {
		if (Control(i, 0) == ToFromAt(0, 0)) {
			EA = Control(i, 1);
			NA = Control(i, 2);
		}

		if (Control(i, 0) == ToFromAt(1, 0)) {
			EB = Control(i, 1);
			NB = Control(i, 2);
		}

		if (Control(i, 0) == ToFromAt(2, 0)) {
			EC = Control(i, 1);
			NC = Control(i, 2);
		}
	}

	for (int i = 0; i < Approx.rows(); i++) {
		if (Approx(i, 0) == ToFromAt(0, 0)) {
			EA = Approx(i, 1);
			NA = Approx(i, 2);
		}

		if (Approx(i, 0) == ToFromAt(1, 0)) {
			EB = Approx(i, 1);
			NB = Approx(i, 2);
		}

		if (Approx(i, 0) == ToFromAt(2, 0)) {
			EC = Approx(i, 1);
			NC = Approx(i, 2);
		}
	}

	double w = value + 180 / PI * (atan2((EA - EC), (NA - NC)) - atan2((EB - EC), (NB - NC))); //The use of ATAN2 already accounts the direction of the angles, and adds the needed constant.
	return w;
}

//Print Matricies to file Matrix
void PrintMatrix(string filename, MatrixXd A,string A1, MatrixXd W, string w1, vector<string> delta, string delta1) {
	ofstream file_out;
	file_out.open(filename, ofstream::out);
	if (file_out.fail()) {
		cout << "Could not write to specified file.";
		exit(0);
	}
	file_out <<fixed<<setprecision(5);
	file_out << A1 << endl;
	for (int i = 0; i < A.rows(); i++) {
		for (int j = 0; j < A.cols(); j++) {
			file_out << A(i, j) << " ";

		}		file_out << endl;
	}
	file_out << w1 << endl;
	for (int i = 0; i < W.rows(); i++) {
		for (int j = 0; j < W.cols(); j++) {
			file_out << W(i, j) << " ";

		}		file_out << endl;
	}

	file_out << delta1 << endl;
	for (int i = 0; i < delta.size(); i++) {
		file_out << delta.at(i) << endl;
		}	file_out.clear();
	}


