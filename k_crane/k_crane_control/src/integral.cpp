#include <iostream>
#include <fstream>
#include </home/aisl/catkin_ws/src/crane/crane_simulator/fanda/include/fanda/Csv.hpp>
#include </home/aisl/catkin_ws/src/crane/crane_simulator/gpop/include/Gpop/Series.hpp>

int main(int argc, char const* argv[])
{
	CSV::CsvFile csv("/home/harumo/catkin_ws/src/k_crane/data/IMU_experiment/input_data/nonlinear_input_velocity.csv");
	if (!csv.is_open()) {
		std::cout << "||| Error. file could not open" << std::endl;
		return 1;
	}
	std::cout << "size : " << csv.collumn_size() << std::endl;
	csv.print();

	std::ofstream file("/home/harumo/catkin_ws/src/k_crane/data/IMU_experiment/input_data/nonlinear_input_position.csv");
	double current_angle = 0;
	std::vector<double> angles;
	double T = 1.0/100.0;
	angles.push_back(0);
	for (int i = 0; i < csv.collumn_size(); i++) {
		double diff_angle;
		try {
			diff_angle = csv(i,0).get_as_double() * T;
		} catch (std::exception ex) {
			std::cout << "iter : " << i << " " << ex.what() << std::endl;
		}
		current_angle = current_angle + diff_angle;
		angles.push_back(current_angle);
		std::cout << "||| current angle : " << current_angle << std::endl;
		file << current_angle << std::endl;
	}

	Gpop::Series plot;
	plot.plot(angles);
	plot.show();
	std::cin.get();
	return 0;
}
