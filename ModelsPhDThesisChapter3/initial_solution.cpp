#include "initial_solution.h"
#include "timetable_data.h"

#include <iostream>
#include <fstream>

namespace alg
{
	bool data_solution_exist;
	std::vector<int> initial_solution_lecture_timing;
	std::vector<int> initial_solution_lecture_room;



	void read_solution_data(const std::string& filename)
	{
		std::ifstream myfile;
		myfile.open(filename);
		if (!myfile.is_open())
			std::cerr << "\n\nError: couldn't open input file for solution data\n\n";
		else
		{
			// clear old data
			clear_solution_data();

			initial_solution_lecture_timing.reserve(nb_sessions);
			initial_solution_lecture_room.reserve(nb_sessions);
			for (int l = 0; l < nb_sessions; ++l)
			{
				int time, location;

				myfile >> time;
				initial_solution_lecture_timing.push_back(time);

				myfile >> location;
				initial_solution_lecture_room.push_back(location);
			}
		}
	}


	void clear_solution_data()
	{
		data_solution_exist = false;
		initial_solution_lecture_timing.clear();
		initial_solution_lecture_room.clear();
	}
}