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

std::string algorithm_type[5] = {"dp","operb","operba","fbqs","sim"};
int main(int argc,char *argv[]){
    streambuf* stream_buffer_cout = cout.rdbuf();
	std::cout << "Start" << std::endl;
    if(argc < 4){
        if (argc<2) return 0;
        if (argv[1] != string("compare")) return 0;
    }

    if (argv[1] == string("compare")) {
		std::cout << "Start compare" << std::endl;
        int size = std::stoi(argv[2]);
        int total_point = 0;
        
        fstream file;
        file.open("result_compare.csv", ios::out);
        string line;

        // Backup streambuffers of  cout
        streambuf* stream_buffer_cout = cout.rdbuf();
        streambuf* stream_buffer_cin = cin.rdbuf();

        // Get the streambuffer of the file
        streambuf* stream_buffer_file = file.rdbuf();

        // Redirect cout to file
        cout.rdbuf(stream_buffer_file);

        
        

		std::cout << "error_bound,FBQS,SIM" << std::endl;

        

        // Redirect cout back to screen
        /*cout.rdbuf(stream_buffer_cout);
        file.close();*/

        for (double error_bound : TEST_ERROR_BOUNDS ){
            //cout << error_bound << endl;
            Algorithm* ptas[2];
            ptas[0] = new FBQS{ error_bound };
            ptas[1] = new SIM{ error_bound };
			double compress_ratio[2];
			Algorithm* pta = nullptr;
            for (int i = 0;i < 2;i++) {
                //cout << "compress" << i << endl;
				pta = ptas[i];
                
                double tx, ty, tt;

                double averge_rate, temp_rate;
                averge_rate = 0;


                double start_time = clock();

                for (int i = 1; i < size + 1; i++) {
                    Trajectory<Point>* traj = new Trajectory<Point>;

                    std::string file_name = "../../dataset/taxi_log_2008_by_id/" + std::to_string(i) + ".txt";
                    /*std::cout << file_name << std::endl;
                    std::cout << "reading file" << endl;*/
                    ifstream file(file_name);

                    int count = 0;
                    string text;
                    while (getline(file, text)) {
                        size_t pos = 0;
                        string delimiter = ",";
                        string x_str, y_str;
                        while ((pos = text.find(delimiter)) != std::string::npos) {
                            x_str = text.substr(0, pos);
                            text.erase(0, pos + delimiter.length());
                        }

                        y_str = text;
                        tx = stod(x_str);
                        ty = stod(y_str);
                        traj->push(Point{ tx,ty,0 });
                        count += 1;
                        //cout << "Point " << count << ": " << Point_2(x, y) << endl;
                    }
                    /*while(scanf("%lf %lf %lf",&tt,&ty,&tx) == 3){
                        traj->push(Point{tx,ty,tt});
                        count += 1;
                    }*/
                    total_point += traj->size();

                    /*std::cout << "Running on No." << i << " trajectory..." << std::endl;
                    std::cout << "Trajectory size:" << traj->size() << std::endl;

                    cout << "start compress" << endl;*/
                    //double start_time = clock();
                    if (traj->size() == 0)continue;

                    auto result = pta->compress(traj);

                    //double end_time = clock();
                   /* cout << "finish compress" << endl;*/

                    //average_second += (double)(end_time - start_time) / CLOCKS_PER_SEC;

                    temp_rate = (double)(traj->size() - result->size() - 1) / traj->size();
                    averge_rate += temp_rate;

                   /* std::cout << "Compressed trajectory size:" << result->size() + 1 << std::endl;
                    std::cout << "Compression rate:" << temp_rate << std::endl;*/

                    delete traj;
                    fclose(stdin);

                }

                double end_time = clock();

                averge_rate /= size;
				compress_ratio[i] = 1.0 - averge_rate ;
                

            }
            
            //file.open("result_compare.csv.txt", ios::app);
            //string line;

            //// Backup streambuffers of  cout
            //streambuf* stream_buffer_cout = cout.rdbuf();

            //// Get the streambuffer of the file
            //streambuf* stream_buffer_file = file.rdbuf();

            //// Redirect cout to file
            //cout.rdbuf(stream_buffer_file);




            std::cout << error_bound << "," << compress_ratio[0] << "," << compress_ratio[1] << std::endl;



            

        }
        // Redirect cout back to screen
        cout.rdbuf(stream_buffer_cout);
        file.close();
		return 0;
		
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

        for (int i = 1; i < size + 1; i++) {
            Trajectory<Point>* traj = new Trajectory<Point>;

            std::string file_name = "../../dataset/taxi_log_2008_by_id/" + std::to_string(i) + ".txt";
            std::cout << file_name << std::endl;
            cout << "reading file" << endl;
            ifstream file(file_name);

            int count = 0;
            string text;
            while (getline(file, text)) {
                size_t pos = 0;
                string delimiter = ",";
                string x_str, y_str;
                while ((pos = text.find(delimiter)) != std::string::npos) {
                    x_str = text.substr(0, pos);
                    text.erase(0, pos + delimiter.length());
                }

                y_str = text;
                tx = stod(x_str);
                ty = stod(y_str);
                traj->push(Point{ tx,ty,0 });
                count += 1;
                //cout << "Point " << count << ": " << Point_2(x, y) << endl;
            }
            /*while(scanf("%lf %lf %lf",&tt,&ty,&tx) == 3){
                traj->push(Point{tx,ty,tt});
                count += 1;
            }*/
            fclose(stdin);

            fstream in_file;
            in_file.open("../../input/" + string(argv[3]) + '/' + std::to_string(i) + ".txt", ios::out);
            string line;

            // Backup streambuffers of  cout
            streambuf* stream_buffer_cout = cout.rdbuf();
            streambuf* stream_buffer_cin = cin.rdbuf();

            // Get the streambuffer of the file
            streambuf* stream_buffer_in_file = in_file.rdbuf();

            // Redirect cout to file
            cout.rdbuf(stream_buffer_in_file);

            //write the trajectory to file
            for (int i = 0; i < traj->size(); i++) {
                cout << (*traj)[i].x << " " << (*traj)[i].y << endl;
            }


            cout.rdbuf(stream_buffer_cout);
            in_file.close();


            total_point += traj->size();

            std::cout << "Running on No." << i << " trajectory..." << std::endl;
            std::cout << "Trajectory size:" << traj->size() << std::endl;
            if (traj->size() == 0)continue;

            cout << "start compress" << endl;
            double start_time = clock();

            auto result = pta->compress(traj);

            double end_time = clock();
            cout << "finish compress" << endl;

            /*average_second += (double)(end_time - start_time) / CLOCKS_PER_SEC;

            temp_rate = (double) (traj->size() - result->size() - 1)/ traj->size();
            averge_rate += temp_rate;

            std::cout << "Compressed trajectory size:" << result->size() + 1 << std::endl;
            std::cout << "Compression rate:" << temp_rate << std::endl;*/

			// if folder not exist, create it
			string folder = "../../output/" + string(argv[3]) + '/';
			

            fstream out_file;
            out_file.open("../../output/" + string(argv[3]) + '/'+to_str(error_bound)+'/' + std::to_string(i) + ".txt", ios::out);
            //string line;

            //// Backup streambuffers of  cout
            //streambuf* stream_buffer_cout = cout.rdbuf();
            //streambuf* stream_buffer_cin = cin.rdbuf();

            //// Get the streambuffer of the file
            streambuf* stream_buffer_out_file = out_file.rdbuf();

            // Redirect cout to file
            cout.rdbuf(stream_buffer_out_file);

            //write the trajectory to file
            for (int i = 0; i < result->size(); i++) {
                cout << (*result)[i].start_point().x << " " << (*result)[i].start_point().y << " " << (*result)[i].end_point().x << " " << (*result)[i].end_point().y << endl;
            }


            cout.rdbuf(stream_buffer_cout);
            out_file.close();
            delete traj;


        }

        /*double end_time = clock();

        averge_rate /= size;
        freopen("result.txt", "w", stdout);

        std::cout << "Error Bound: " << error_bound << "m" << std::endl;
        std::cout << "Average Compression Ratio: " << 100.0 - averge_rate * 100 << "\%" << std::endl;
        std::cout << "Running time " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << std::endl;
        std::cout << "Average Second:" << average_second / (double)size << std::endl;
        fclose(stdout);*/

    }
    return 0;
}