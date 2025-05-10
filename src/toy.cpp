#include <iostream>
#include <cstdio>
#include <ctime>
#include <string>
#include <fstream>
#include <sstream>
#include <string>
#include "../inc/trajectory.hpp"
#include "../inc/operb.hpp"
#include "../inc/algorithm.hpp"
#include "../inc/dp.hpp"
#include "../inc/fbqs.hpp"
#include "../inc/sim.hpp"

//#pragma comment(linker, "/STACK:1024000000,1024000000") 

using namespace std;

const double TEST_ERROR_BOUNDS[] = { 1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 60.0, 80.0, 100.0 };


template < typename Type > std::string to_str(const Type& t)
{
    std::ostringstream os;
    os << t;
    return os.str();
}

std::string algorithm_type[5] = { "dp","operb","operba","fbqs","sim" };
int main(int argc, char* argv[]) {
    streambuf* stream_buffer_cout = cout.rdbuf();
    std::cout << "Start" << std::endl;
    if (argc < 4) {
        if (argc < 2) return 0;
        if (argv[1] != string("compare")) return 0;
    }

    

    double error_bound_input = std::stod(argv[1]);
    int size = std::stoi(argv[2]);
    int total_point = 0;
    Algorithm* pta = nullptr;
    double average_second = 0.0;

    vector<double> error_bounds;
    if (error_bound_input == -1) error_bounds = { 0.001, 0.01, 0.1, 1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 60.0, 80.0, 100.0, 1000.0,10000.0 };
    else error_bounds.push_back(error_bound_input);
    for (double error_bound : error_bounds) {

        if (argv[3] == algorithm_type[0]) {
            pta = new DP{ error_bound };

        }
        else if (argv[3] == algorithm_type[1]) {
            pta = new OPERB{ error_bound };
        }
        else if (argv[3] == algorithm_type[2]) {
            pta = new OPERBA{ error_bound };
        }
        else if (argv[3] == algorithm_type[3]) {
            pta = new FBQS{ error_bound };
        }

        else if (argv[3] == algorithm_type[4]) {
            pta = new SIM{ error_bound };
        }

        //int traj_size = std::stoi(argv[4]);

        double tx, ty, tt;

        double averge_rate, temp_rate;
        averge_rate = 0;


        double start_time = clock();

        
            Trajectory<Point>* traj = new Trajectory<Point>;

            

			traj->push(Point{ 0,0});
			traj->push(Point{ 1,1});
			traj->push(Point{ 2,0.01 });
			traj->push(Point{ 3,1});



            

            std::cout << "Trajectory size:" << traj->size() << std::endl;
            if (traj->size() == 0)continue;

            cout << "start compress" << endl;

            auto result = pta->compress(traj);

            double end_time = clock();
            cout << "finish compress" << endl;
			cout << "Compressed trajectory size:" << result->size() << std::endl;
            //average_second += (double)(end_time - start_time) / CLOCKS_PER_SEC;
			for (int i = 0; i < result->size(); i++) {
				cout << (*result)[i].start_point().x << " " << (*result)[i].start_point().y << " " << (*result)[i].end_point().x << " " << (*result)[i].end_point().y << endl;
			}

            temp_rate = (double)(traj->size() - result->size()) / traj->size();
            
            averge_rate += temp_rate;

            
            delete traj;


        

        //double end_time = clock();

        
        //freopen("result.txt", "w", stdout);

        std::cout << "Error Bound: " << error_bound << "m" << std::endl;
        std::cout << "Average Compression Ratio: " << 100.0 - averge_rate * 100 << "\%" << std::endl;
        //std::cout << "Running time " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
        //std::cout << "Average Second:" << average_second / (double)size << std::endl;
        //fclose(stdout);

    }
    return 0;
}