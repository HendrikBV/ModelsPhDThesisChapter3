#include "timetable_data.h"
#include "building_data.h"
#include "initial_solution.h"

#include "timetable_instance_generator.h"

#include "MIP_monolithic.h"
#include "MIP_two_stage.h"
#include "MIP_only_second_stage.h"
#include "heuristic.h"

#include <iostream>
#include <stdexcept>



int main()
{
	for (int instance = 0; instance < 8; ++instance)
	{
		try
		{
			// Read timetable data
			std::string filename_timetabledata;
			if (instance == 0)
				filename_timetabledata = "Instances/KUL_CH3.txt";
			else
				filename_timetabledata = "Instances/Comp0" + std::to_string(instance) + ".txt";
			alg::read_timetable_data(filename_timetabledata);

			// Read building data
			std::string filename_buildingdata;
			if (alg::nb_locations == 8)
				filename_buildingdata = "Instances/Building_B_8_2_CH3.txt";
			else if (alg::nb_locations == 16)
				filename_buildingdata = "Instances/Building_B_16_1_CH3.txt";
			else if (alg::nb_locations == 20)
				filename_buildingdata = "Instances/Building_B_20_CH3.txt";
			else if (alg::nb_locations == 56)
				filename_buildingdata = "Instances/Building_KUL_CH3.txt";
			alg::read_building_data(filename_buildingdata);

			// Run algorithm 5x
			for (int it = 0; it < 5; ++it)
			{
				try
				{
					alg::heuristic_master heuristic;
					alg::heuristic_master::_penalty_value_constraint_violation = 100000;
					alg::heuristic_master::_lambda = 0;
					alg::heuristic_master::_time_limit = 1000;
					alg::heuristic_master::_SA_start_temperature = 100000;
					alg::heuristic_master::_SA_alpha = 0.9;
					alg::heuristic_master::_SA_Tmin = 0.1;
					alg::heuristic_master::_nb_synchronizations = 40;
					alg::heuristic_master::_nb_threads = 12;

					heuristic.run();
				}
				catch (const std::exception& ex)
				{
					std::cout << "\n\n" << ex.what();
				}
			}
		}
		catch (const std::exception& ex)
		{
			std::cout << "\n\n" << ex.what();
		}
	}

	std::cout << "\n\n\n\n\nPress enter to exit ...";
	getchar();
	return 0;
}

