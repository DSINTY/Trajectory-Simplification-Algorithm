#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Frechet_distance.h>
//#include <CGAL/IO/WKT.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>

#include <ostream>
#include <fstream>
#include <string>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;

using namespace std;

const string INPUT_PATH = "../../input/fbqs/";
const string OUTPUT_PATH = "../../output/";
int NUM_FILES = 100;
int ERROR_BOUNDS[] = { 1, 2, 5, 10, 20, 40, 60, 80, 100, 1000, 10000 };

string SIM_ALGS[] = {"sim", "fbqs"};

int main(int argc, char* argv[])
{
    fstream write_file;
    write_file.open("result_compare.csv", ios::out);
    string write_line;

    // Backup streambuffers of  cout
    streambuf* stream_buffer_cout = cout.rdbuf();
    streambuf* stream_buffer_cin = cin.rdbuf();

    // Get the streambuffer of the file
    streambuf* stream_buffer_file = write_file.rdbuf();

    // Redirect cout to file
    cout.rdbuf(stream_buffer_file);
    cout << "ID,original_length";
    for (int error_bound : ERROR_BOUNDS) {
		for (string sim_alg : SIM_ALGS) {
			cout << ",rate_" << sim_alg << "_" << error_bound;
			cout << ",dist_" << sim_alg << "_" << error_bound;
		}
    }
	cout << endl;

    for (int i = 1;i <= NUM_FILES;i++) {
		string input_file_name = INPUT_PATH + to_string(i) + ".txt";
		ifstream input_file(input_file_name);
        string input_line;
		vector<Point_2> original_traj;
		while (getline(input_file, input_line)) {
			size_t pos = 0;
			string delimiter = " ";
			string x_str, y_str;
			while ((pos = input_line.find(delimiter)) != std::string::npos) {
				x_str = input_line.substr(0, pos);
                input_line.erase(0, pos + delimiter.length());
			}
			y_str = input_line;
			Point_2 p{ stod(x_str), stod(y_str) };
            original_traj.push_back(p);
		}
		input_file.close();
		cout << i << "," << original_traj.size();
        for (int error_bound : ERROR_BOUNDS) {
            for (string sim_alg : SIM_ALGS) {

				string output_file_name = OUTPUT_PATH + sim_alg + "/" + to_string(error_bound) + "/" + to_string(i) + ".txt";
				ifstream output_file(output_file_name);
				string output_line;
				vector<Point_2> compressed_traj;
				while (getline(output_file, output_line)) {
					size_t pos = 0;
					string delimiter = " ";
					string inputs[4];
					int j = 0;
					while ((pos = output_line.find(delimiter)) != std::string::npos) {
						inputs[j] = output_line.substr(0, pos);
						output_line.erase(0, pos + delimiter.length());
						j++;
					}
					inputs[j] = output_line;
					double start_x = stod(inputs[0]);
					double start_y = stod(inputs[1]);
					double end_x = stod(inputs[2]);
					double end_y = stod(inputs[3]);
					Point_2 p1{ start_x, start_y };
					Point_2 p2{ end_x, end_y };
					if (compressed_traj.size() == 0) {
						compressed_traj.push_back(p1);
					}
					else if (compressed_traj[compressed_traj.size() - 1] != p1) {
						compressed_traj.push_back(p1);
					}
					if (compressed_traj[compressed_traj.size() - 1] != p2) {
						compressed_traj.push_back(p2);
					}
					

				}
				

				output_file.close();
				double compress_rate = (double) (double (compressed_traj.size())  /double (original_traj.size()));
				cout << "," << compress_rate;

				pair<double,double> dists = CGAL::bounded_error_Frechet_distance(original_traj, compressed_traj, 0.001);
				double dist = (dists.first+dists.second)/2;


				cout << "," << dist;

            }
        }
		cout << endl;
    }


    cout.rdbuf(stream_buffer_cout);
    write_file.close();
	return 0;
}


