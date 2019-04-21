#include "heuristic.h"
#include "timetable_data.h"
#include "building_data.h"
#include "initial_solution.h"
#include "logger.h"
#include "MIP_two_stage.h"

#include <iostream>
#include <random>
#include <utility>
#include <thread>
#include <future>
#include <mutex>
#include <string>



namespace
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Random number generator and move probabilities
	std::random_device randdev;
	std::seed_seq seedseq{ randdev(),randdev(),randdev(),randdev(),randdev(),randdev(),randdev() };
	std::mt19937_64 generator(seedseq);



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Sorting function
	bool sort_on_room(const std::pair<int, int>& a, const std::pair<int, int>& b)
	{
		if (a.second < b.second)
			return true;

		return false;
	}
}





namespace alg
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/*!
	*	@brief	Evaluate the objective value of a solution.
	*	@param	solution	The solution that is evaluated.
	*	@param	info		To store information on the objective value.
	*/
	void evaluate(const alg::matrix2D<int>& solution, alg::information_objective_value& info)
	{
		// A. TIMETABLE 
		// I. CONSTRAINTS
		// 1. No conflicts
		info.constraint_violations_scheduling_conflicts = 0;
		for (int t = 0; t < nb_timeslots; ++t)
		{
			int violations = 0;
			for (int r1 = 0; r1 < nb_locations; ++r1)
			{
				for (int r2 = r1 + 1; r2 < nb_locations; ++r2)
				{
					if (solution.at(t, r1) >= 0 && solution.at(t, r2) >= 0 && get_sessionsessionconflict(solution.at(t, r1), solution.at(t, r2)))
						++violations;
				}
			}
			info.constraint_violations_scheduling_conflicts += violations;
		}

		// 2. Correct rooms
		info.constraint_violations_correct_room = 0;
		for (int t = 0; t < nb_timeslots; ++t)
		{
			int violations = 0;
			for (int r = 0; r < nb_locations; ++r)
			{
				int lec = solution.at(t, r);
				if (lec >= 0 && !get_sessionlocationpossible(lec, r))
					++violations;

			}
			info.constraint_violations_correct_room += violations;
		}

		// 3. Teacher working time regulations
		info.constraint_violations_teacher_working_time = 0;

		// 3.A teacher working time regulations: at most 4 lectures per day
		for (int d = 0; d < nb_teachers; ++d)
		{
			for (int day = 0; day < nb_days; ++day)
			{
				int first_timeslot = day * nb_timeslots_per_day;
				int last_timeslot = (day + 1) * nb_timeslots_per_day;

				int teaches = 0;
				for (int t = first_timeslot; t < last_timeslot; ++t)
				{
					for (int r = 0; r < nb_locations; ++r)
					{
						int lec = solution.at(t, r);
						if (lec >= 0 && get_teachersession(d, lec))
							++teaches;
					}
				}
				if (teaches > 4)
					++info.constraint_violations_teacher_working_time;
			}
		}

		// 3.B teacher working time regulations: no more than 3 lectures consecutively
		for (int d = 0; d < nb_teachers; ++d)
		{
			for (int day = 0; day < nb_days; ++day)
			{
				int first_timeslot = day * nb_timeslots_per_day;
				int last_timeslot = first_timeslot + nb_timeslots_per_day - 3;

				int teaches = 0;
				for (int t = first_timeslot; t < last_timeslot; ++t)
				{
					bool teaches_ts1 = false, teaches_ts2 = false, teaches_ts3 = false, teaches_ts4 = false;
					for (int r = 0; r < nb_locations; ++r)
					{
						int lec1 = solution.at(t, r);
						int lec2 = solution.at(t + 1, r);
						int lec3 = solution.at(t + 2, r);
						int lec4 = solution.at(t + 3, r);

						if (lec1 >= 0 && get_teachersession(d, lec1))
							++teaches_ts1 = true;
						if (lec2 >= 0 && get_teachersession(d, lec2))
							++teaches_ts2 = true;
						if (lec3 >= 0 && get_teachersession(d, lec3))
							++teaches_ts3 = true;
						if (lec4 >= 0 && get_teachersession(d, lec4))
							++teaches_ts4 = true;
					}
					if (teaches_ts1 && teaches_ts2 && teaches_ts3 && teaches_ts4)
						++info.constraint_violations_teacher_working_time;
				}
			}
		}

		// 3.C teacher working time regulations: not last timeslot of previous day and first timeslot of next day
		for (int d = 0; d < nb_teachers; ++d)
		{
			for (int day = 1; day < nb_days; ++day)
			{
				int first_timeslot_of_next_day = day * nb_timeslots_per_day;
				int last_timeslot_of_previous_day = day * nb_timeslots_per_day - 1;

				bool teaches_t1 = false, teaches_t2 = false;
				for (int r = 0; r < nb_locations; ++r)
				{
					int lec1 = solution.at(last_timeslot_of_previous_day, r);
					if (lec1 >= 0 && get_teachersession(d, lec1))
						teaches_t1 = true;

					int lec2 = solution.at(first_timeslot_of_next_day, r);
					if (lec2 >= 0 && get_teachersession(d, lec2))
						teaches_t2 = true;
				}
				if (teaches_t1 && teaches_t2)
					++info.constraint_violations_teacher_working_time;
			}
		}

		// 3.D teacher working time regulations: not first and last timeslot of same day
		for (int d = 0; d < nb_teachers; ++d)
		{
			for (int day = 0; day < nb_days; ++day)
			{
				int first_timeslot_of_day = day * nb_timeslots_per_day;
				int last_timeslot_of_day = (day + 1) * nb_timeslots_per_day - 1;

				bool teaches_t1 = false, teaches_t2 = false;
				for (int r = 0; r < nb_locations; ++r)
				{
					int lec1 = solution.at(last_timeslot_of_day, r);
					if (lec1 >= 0 && get_teachersession(d, lec1))
						teaches_t1 = true;

					int lec2 = solution.at(first_timeslot_of_day, r);
					if (lec2 >= 0 && get_teachersession(d, lec2))
						teaches_t2 = true;
				}
				if (teaches_t1 && teaches_t2)
					++info.constraint_violations_teacher_working_time;
			}
		}

		// 4. Compactness constraints
		if (nb_timeslots_per_day == 5 || nb_timeslots_per_day == 6)
		{
			info.constraint_violations_compactness_constraints = 0;
			for (int s = 0; s < nb_series; ++s)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					for (int t = 0; t < 3; ++t) // (1-3, 2-4,) 3-5
					{
						int ts1 = day * nb_timeslots_per_day + t;
						int ts2 = day * nb_timeslots_per_day + t + 1;
						int ts3 = day * nb_timeslots_per_day + t + 2;

						bool classts1 = false, classts2 = false, classts3 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							if (solution.at(ts1, r) >= 0 && get_seriessession(s, solution.at(ts1, r)))
								classts1 = true;
							if (solution.at(ts2, r) >= 0 && get_seriessession(s, solution.at(ts2, r)))
								classts2 = true;
							if (solution.at(ts3, r) >= 0 && get_seriessession(s, solution.at(ts3, r)))
								classts3 = true;
						}

						if (classts1 && !classts2 && classts3)
							++info.constraint_violations_compactness_constraints;
					}
				}
			}
		}
		else
		{
			// N/A 
			info.constraint_violations_compactness_constraints = 0;
		}



		// II. OBJECTIVE (PREFERENCES)
		info.preference_score = 0;
		for (int t = 0; t < nb_timeslots; ++t)
		{
			int pref = 0;
			for (int r = 0; r < nb_locations; ++r)
			{
				int lec = solution.at(t, r);
				if (lec >= 0)
					pref += (get_costsessiontimeslot(lec, t) + 1000 * get_costsession_ts_educational(lec, t));
			}
			info.preference_score += pref;
		}
		if (heuristic_master::_constraint_preferences && info.preference_score > heuristic_master::_constraint_preferences_value)
			info.preference_score += heuristic_master::_penalty_value_constraint_violation;



		// B. FLOWS
		// I. Evacuations
		double objective_evacuations = 0.0;
		{
			info.evacuations_timeslot_lecture_uses_path.fill(0.0);
			info.evacuations_timeslot_flow_per_arc.fill(0.0);
			info.evacuations_timeslot_time_per_arc.fill(0.0);
			info.evacuations_timeslot_lecture_travel_time.fill(0.0);
			for (int t = 0; t < nb_timeslots; ++t)
				info.evacuations_timeslot_max_travel_time.at(t) = 0.0;

			for (int t = 0; t < nb_timeslots; ++t)
			{
				//std::cout << "\n\n\n\n\n\nTIMESLOT: " << t + 1;

				// 1. who uses which paths
				for (int r = 0; r < nb_locations; ++r)
				{
					int lec = solution.at(t, r);
					if (lec >= 0)
					{
						for (int p = 0; p < nb_paths; ++p)
							info.evacuations_timeslot_lecture_uses_path.at(t, lec, p) = get_roomroompath(r, nb_locations, p);
					}
				}

				// 2. flow per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					for (int l = 0; l < nb_sessions; ++l)
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							double nbp = info.evacuations_timeslot_lecture_uses_path.at(t, l, p) * session_nb_people.at(l);
							if (get_patharc(p, ij))
								info.evacuations_timeslot_flow_per_arc.at(t, ij) += nbp;
						}
					}
				}
				//std::cout << "\n\n\nevacuations_timeslot_flow_per_arc\n\n";
				//evacuations_timeslot_flow_per_arc.print();

				// 3. time per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					info.evacuations_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * info.evacuations_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
					if (arc_stairs.at(ij))
						info.evacuations_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
				}
				//std::cout << "\n\n\nevacuations_timeslot_time_per_arc\n\n";
				//evacuations_timeslot_time_per_arc.print();

				// 4. time per lecture in total
				for (int l = 0; l < nb_sessions; ++l)
				{
					double time_longest_path = 0.0;
					for (int p = 0; p < nb_paths; ++p)
					{
						if (info.evacuations_timeslot_lecture_uses_path.at(t, l, p) > 0.01)
						{
							double time_current_path = 0.0;
							for (int ij = 0; ij < nb_arcs; ++ij)
							{
								if (get_patharc(p, ij))
									time_current_path += info.evacuations_timeslot_time_per_arc.at(t, ij);
							}
							if (time_current_path > time_longest_path)
								time_longest_path = time_current_path;
						}
					}
					info.evacuations_timeslot_lecture_travel_time.at(t, l) = time_longest_path;
				}
				//std::cout << "\n\n\nevacuations_timeslot_lecture_travel_time\n\n";
				//evacuations_timeslot_lecture_travel_time.print();

				// 5. maximum evacuation time per timeslot
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (info.evacuations_timeslot_lecture_travel_time.at(t, l) > info.evacuations_timeslot_max_travel_time.at(t))
						info.evacuations_timeslot_max_travel_time.at(t) = info.evacuations_timeslot_lecture_travel_time.at(t, l);
				}
			}

			// 6. objective evacuations
			if (heuristic_master::_objective_type == heuristic_master::objective_type::maximum_over_timeslots)
			{
				for (int t = 0; t < nb_timeslots; ++t)
					if (info.evacuations_timeslot_max_travel_time.at(t) > objective_evacuations)
						objective_evacuations = info.evacuations_timeslot_max_travel_time.at(t);
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				for (int t = 0; t < nb_timeslots; ++t)
					objective_evacuations += info.evacuations_timeslot_max_travel_time.at(t);
			}
		}


		// Total objective value
		info.objective_value = (heuristic_master::_penalty_value_constraint_violation * (info.constraint_violations_scheduling_conflicts
			+ info.constraint_violations_correct_room
			+ info.constraint_violations_teacher_working_time
			+ info.constraint_violations_compactness_constraints)
			+ alg::heuristic_master::_lambda * info.preference_score + (1 - alg::heuristic_master::_lambda) * objective_evacuations);
	}



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	double heuristic_master::_probability_move[3] = { 0.6, 0.38, 0.02 };
	bool heuristic_master::_constraint_preferences = false;
	bool heuristic_master::_constraint_preferences_value = 1e20;
	double heuristic_master::_lambda = 0;
	double heuristic_master::_time_limit = 1000;
	heuristic_master::objective_type heuristic_master::_objective_type = heuristic_master::objective_type::sum_over_timeslots;
	double heuristic_master::_penalty_value_constraint_violation = 10000;
	size_t heuristic_master::_nb_threads = 8;
	size_t heuristic_master::_nb_synchronizations = 40;
	double heuristic_master::_SA_temperature = 1;
	double heuristic_master::_SA_alpha = 0.5;
	double heuristic_master::_SA_start_temperature = 1;
	double heuristic_master::_SA_Tmin = 0.1;



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void heuristic_master::write_output()
	{
		std::string text;

		// 1. instance name
		text = "Algorithm: "; text.append(algorithm_name);
		text += "\nProblem: " + instance_name_timetable + " + " + instance_name_building;

		// 2. settings
		text += "\nSettings:";
		text += "\n\tLambda: " + std::to_string(_lambda);
		//text += "\n\tAlpha: " + std::to_string(_alpha);
		text += "\n\tObjective type: ";
		if (_objective_type == objective_type::maximum_over_timeslots)
			text += "maximum over timeslots";
		else
			text += "sum over timeslots";
		text += "\n\tTime limit (seconds): " + std::to_string(_time_limit);
		text += "\nS.A. parameters:";
		text += "\n\tInitial temperature: " + std::to_string(_SA_start_temperature);
		text += "\n\tTemperature alpha: " + std::to_string(_SA_alpha);
		text += "\n\tTmin: " + std::to_string(_SA_Tmin);
		text += "\nMultithreading parameters: ";
		text += "\n\tNumber of threads: " + std::to_string(_nb_threads);
		text += "\n\tNumber of synchronizations: " + std::to_string(_nb_synchronizations);

		// 4. results
		text += "\nIterations: " + std::to_string(total_iterations);
		text += "\nReheats: " + std::to_string(reheats);
		text += "\nObjective value best found solution: " + std::to_string(best_objective);

		// 5. solution
		/*text += "\nSolution:";
		for (int t = 0; t < nb_timeslots; ++t)
		{
		text += "\nTimeslot: " + std::to_string(t + 1);
		for (int l = 0; l < nb_sessions; ++l)
		{
		for (int r = 0; r < nb_locations; ++r)
		{
		if (best_solution.at(t, r) == l)
		text += "\n\tLecture " + std::to_string(l + 1) + " is planned in timeslot " + std::to_string(t + 1) + " and room " + std::to_string(r + 1);
		}
		}
		}*/


		// Write single item to logger
		_logger << logger::log_type::INFORMATION << text;
	}



	void heuristic_master::run()
	{
		// 1. start timer
		std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

		// 2. constructive method: MIP
		{
			MIP_two_stage MIP;
			MIP.run_only_first_stage();
			if (!MIP.get_solution_firststage_exists())
				throw std::runtime_error("Error in function heuristic::run(). \nCouldn't construct a feasible solution.");

			best_solution.set(nb_timeslots, nb_locations, -1);
			for (int l = 0; l < nb_sessions; ++l)
			{
				int timeslot = MIP.get_solution_lecture_timeslot(l);
				int room = MIP.get_solution_lecture_room(l);

				best_solution.at(timeslot, room) = l;
			}
		}
		/*{ // Random solution
		best_solution.set(nb_timeslots, nb_locations, -1);
		int l = 0;
		for (int t = 0; t < nb_timeslots; ++t)
		{
		for (int r = 0; r < nb_locations; ++r)
		{
		if (l >= nb_sessions)
		break;

		best_solution.at(t, r) = l;
		++l;
		}
		}
		}*/
		best_solution.print();

		information_objective_value info_objval;
		{
			info_objval.evacuations_timeslot_lecture_uses_path.set(nb_timeslots, nb_sessions, nb_paths, 0.0);
			info_objval.evacuations_timeslot_flow_per_arc.set(nb_timeslots, nb_arcs, 0.0);
			info_objval.evacuations_timeslot_time_per_arc.set(nb_timeslots, nb_arcs, 0.0);
			info_objval.evacuations_timeslot_lecture_travel_time.set(nb_timeslots, nb_sessions, 0.0);
			info_objval.evacuations_timeslot_max_travel_time.reserve(nb_timeslots);
			for (int i = 0; i < nb_timeslots; ++i)
				info_objval.evacuations_timeslot_max_travel_time.push_back(0.0);
		}
		evaluate(best_solution, info_objval);
		std::string output_text = "\n\nObjective value initial solution: " + std::to_string(info_objval.objective_value) + "\n";
		std::cout << output_text;


		// 3. create subprocesses
		std::vector<std::future<double>> results_threads;
		std::vector<double> objective_values_threads;
		for (int i = 0; i < _nb_threads; ++i)
		{
			threads.push_back(heuristic_subprocess());
			threads.back()._process_id = i;
			threads.back().initialize();
			threads.back().set_solution(best_solution, info_objval);
			results_threads.push_back(std::future<double>());
			objective_values_threads.push_back(1e20);
		}


		// 4. repeat until timeout
		std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;
		double time_per_process = (_time_limit - elapsed_time.count()) / _nb_synchronizations;

		_SA_temperature = _SA_start_temperature;
		while (true)
		{
			// time check
			elapsed_time = std::chrono::system_clock::now() - start_time;

			if (elapsed_time.count() > _time_limit)
			{
				output_text = "\n\nLAHC heuristic has reached time limit of " + std::to_string(_time_limit) + " seconds.";
				std::cout << output_text;
				break;
			}

			// run threads with specified time limit
			for (int i = 0; i < _nb_threads; ++i)
			{
				results_threads.at(i) = std::async(std::launch::async, &heuristic_subprocess::run, &threads.at(i), time_per_process);
			}

			// find best solution over all threads
			int best_index = -1;
			double best_objval = 1e20;
			for (int i = 0; i < _nb_threads; ++i)
			{
				objective_values_threads.at(i) = results_threads.at(i).get();
				if (objective_values_threads.at(i) < best_objval)
				{
					best_objval = objective_values_threads.at(i);
					best_index = i;
				}
			}

			output_text = "\n\n\n\n\nBest solution found by thread " + std::to_string(best_index) + ", with objective value = " + std::to_string(objective_values_threads.at(best_index));
			std::cout << output_text;

			// set current solution of all threads equal to this best found solution
			for (int i = 0; i < _nb_threads; ++i)
			{
				threads.at(i).set_solution(threads.at(best_index).get_best_solution(), threads.at(best_index).get_best_objective_value());
			}

			// update the temperature
			_SA_temperature *= _SA_alpha;
			if (_SA_temperature < _SA_Tmin)
			{
				_SA_temperature = _SA_start_temperature;
				++reheats;
			}

			output_text = "\n\nUpdating temperature: T' = " + std::to_string(_SA_temperature) + "\n\n\n";
			std::cout << output_text;
		}


		// 5. return best found solution
		// find best solution over all threads
		int best_index = -1;
		double best_objval = 1e20;
		total_iterations = 0;
		for (int i = 0; i < _nb_threads; ++i)
		{
			if (objective_values_threads.at(i) < best_objval)
			{
				best_objval = objective_values_threads.at(i);
				best_index = i;
			}

			total_iterations += threads.at(i).get_iterations();
		}

		best_solution = threads.at(best_index).get_best_solution();
		best_objective = threads.at(best_index).get_best_objective_value().objective_value;
		best_solution.print();

		information_objective_value info;
		{
			info.evacuations_timeslot_lecture_uses_path.set(nb_timeslots, nb_sessions, nb_paths, 0.0);
			info.evacuations_timeslot_flow_per_arc.set(nb_timeslots, nb_arcs, 0.0);
			info.evacuations_timeslot_time_per_arc.set(nb_timeslots, nb_arcs, 0.0);
			info.evacuations_timeslot_lecture_travel_time.set(nb_timeslots, nb_sessions, 0.0);
			info.evacuations_timeslot_max_travel_time.reserve(nb_timeslots);
			for (int i = 0; i < nb_timeslots; ++i)
				info.evacuations_timeslot_max_travel_time.push_back(0.0);
		}
		evaluate(best_solution, info);
		info.print();

		// 6. write output
		write_output();
	}



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void heuristic_subprocess::initialize()
	{
		current_objective.evacuations_timeslot_lecture_uses_path.set(nb_timeslots, nb_sessions, nb_paths, 0.0);
		current_objective.evacuations_timeslot_flow_per_arc.set(nb_timeslots, nb_arcs, 0.0);
		current_objective.evacuations_timeslot_time_per_arc.set(nb_timeslots, nb_arcs, 0.0);
		current_objective.evacuations_timeslot_lecture_travel_time.set(nb_timeslots, nb_sessions, 0.0);
		current_objective.evacuations_timeslot_max_travel_time.reserve(nb_timeslots);
		for (int i = 0; i < nb_timeslots; ++i)
			current_objective.evacuations_timeslot_max_travel_time.push_back(0.0);

		best_objective.evacuations_timeslot_lecture_uses_path.set(nb_timeslots, nb_sessions, nb_paths, 0.0);
		best_objective.evacuations_timeslot_flow_per_arc.set(nb_timeslots, nb_arcs, 0.0);
		best_objective.evacuations_timeslot_time_per_arc.set(nb_timeslots, nb_arcs, 0.0);
		best_objective.evacuations_timeslot_lecture_travel_time.set(nb_timeslots, nb_sessions, 0.0);
		best_objective.evacuations_timeslot_max_travel_time.reserve(nb_timeslots);
		for (int i = 0; i < nb_timeslots; ++i)
			best_objective.evacuations_timeslot_max_travel_time.push_back(0.0);
	}



	double heuristic_subprocess::run(double time_limit)
	{
		// 1. start timer
		std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

		// 2. main SA loop
		while (true)
		{
			// time check
			std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;
			if (elapsed_time.count() > time_limit) // stop thread
				return best_objective.objective_value;

			// new iteration
			++_iterations;

			// choose a move
			std::uniform_real_distribution<double> prob_dist(0, 1);
			double prob_move = prob_dist(generator);
			double prob_accept = prob_dist(generator);




			// A. Lecture swap
			if (prob_move < heuristic_master::_probability_move[0])
			{
				int timeslot1 = 0, timeslot2 = 0, room1 = 0, room2 = 0;
				std::uniform_int_distribution<int> dist_timeslot(0, nb_timeslots - 1);
				std::uniform_int_distribution<int> dist_rooms(0, nb_locations - 1);

				timeslot1 = dist_timeslot(generator);
				do
				{
					timeslot2 = dist_timeslot(generator);
				} while (timeslot1 == timeslot2 && nb_timeslots > 1); // if nb_timeslots == 1, lecture_swap is same as room_swap
				room1 = dist_rooms(generator);
				room2 = dist_rooms(generator);

				if (timeslot2 < timeslot1)
					std::swap(timeslot1, timeslot2);

				double candidate_objective = evaluate_incremental(current_solution, timeslot1, timeslot2, room1, room2);
				double percentage_deterioration = (candidate_objective - current_objective.objective_value) / current_objective.objective_value;

				// accept if better than current solution or better than value in list
				if (candidate_objective < current_objective.objective_value
					|| std::exp(-percentage_deterioration / heuristic_master::_SA_temperature) < prob_accept)
				{
					// implement the change
					std::swap(current_solution.at(timeslot1, room1), current_solution.at(timeslot2, room2));

					//current_solution.print();

					// save the objective value
					current_objective.objective_value = candidate_objective;

					// update the values for individual objectives and constraints
					current_objective.constraint_violations_scheduling_conflicts = new_constraint_violations_scheduling_conflicts;
					current_objective.constraint_violations_correct_room = new_constraint_violations_correct_room;
					current_objective.constraint_violations_teacher_working_time = new_constraint_violations_teacher_working_time;
					current_objective.constraint_violations_compactness_constraints = new_constraint_violations_compactness_constraints;
					current_objective.preference_score = new_preference_score;

					// update the evacuation and travel times
					{
						current_objective.evacuations_timeslot_max_travel_time.at(timeslot1) = new_max_evac_time_changed_ts.at(0);
						current_objective.evacuations_timeslot_max_travel_time.at(timeslot2) = new_max_evac_time_changed_ts.at(1);
					}

					// check if better than best solution
					if (current_objective.objective_value < best_objective.objective_value)
					{
						best_objective = current_objective; // save entire POD 
						best_solution = current_solution;
						std::string text = "\n\nNew best solution found (Lecture Swap) in thread " + std::to_string(_process_id) + "!\nObjective value " + std::to_string(current_objective.objective_value);
						std::cout << text;
					}
				}
			}





			// B. Room Swap
			else if (prob_move < heuristic_master::_probability_move[0] + heuristic_master::_probability_move[1])
			{
				int timeslot = 0, room1 = 0, room2 = 0;
				std::uniform_int_distribution<int> dist_timeslot(0, nb_timeslots - 1);
				std::uniform_int_distribution<int> dist_rooms(0, nb_locations - 1);

				timeslot = dist_timeslot(generator);
				room1 = dist_rooms(generator);
				do {
					room2 = dist_rooms(generator);
				} while (room1 == room2);

				double candidate_objective = evaluate_incremental(current_solution, timeslot, room1, room2);
				double percentage_deterioration = (candidate_objective - current_objective.objective_value) / current_objective.objective_value;

				// accept if better than current solution or better than value in list
				if (candidate_objective < current_objective.objective_value
					|| std::exp(-percentage_deterioration / heuristic_master::_SA_temperature) < prob_accept)
				{
					// implement the change
					std::swap(current_solution.at(timeslot, room1), current_solution.at(timeslot, room2));

					// update the objective value
					current_objective.objective_value = candidate_objective;

					// update the values for individual objectives and constraints
					current_objective.constraint_violations_scheduling_conflicts = new_constraint_violations_scheduling_conflicts;
					current_objective.constraint_violations_correct_room = new_constraint_violations_correct_room;
					current_objective.constraint_violations_teacher_working_time = new_constraint_violations_teacher_working_time;
					current_objective.constraint_violations_compactness_constraints = new_constraint_violations_compactness_constraints;
					current_objective.preference_score = new_preference_score;

					// save the new evacuation/travel times
					{
						current_objective.evacuations_timeslot_max_travel_time.at(timeslot) = new_max_evac_time_changed_ts_roomswap;
					}

					if (current_objective.objective_value < best_objective.objective_value)
					{
						best_objective = current_objective;
						best_solution = current_solution;
						std::string text = "\n\nNew best solution found (Room Swap) in thread " + std::to_string(_process_id) + "!\nObjective value " + std::to_string(current_objective.objective_value);
						std::cout << text;

						//best_solution.print();
					}
				}
			}




			// C. Kempe Chain
			else
			{
				// Only makes sense if nb_timeslots > 1
				if (nb_timeslots > 1)
				{
					// (1) first we select two timeslots
					std::uniform_int_distribution<int> dist_timeslot(0, nb_timeslots - 1);
					int timeslot1 = 0, timeslot2 = 0;
					bool ts1_has_lectures = false, ts2_has_lectures = false;

					do
					{
						timeslot1 = dist_timeslot(generator);
						for (int r = 0; r < nb_locations; ++r)
						{
							if (current_solution.at(timeslot1, r) >= 0)
							{
								ts1_has_lectures = true;
								break;
							}
						}
					} while (!ts1_has_lectures);

					do
					{
						timeslot2 = dist_timeslot(generator);
						for (int r = 0; r < nb_locations; ++r)
						{
							if (current_solution.at(timeslot2, r) >= 0)
							{
								ts2_has_lectures = true;
								break;
							}
						}
					} while (timeslot1 == timeslot2 || !ts2_has_lectures);

					if (timeslot2 < timeslot1)
						std::swap(timeslot1, timeslot2);


					// (2) Now we construct our chains
					std::vector<std::pair<int, int>> unassigned_nodes;	// first == timeslot, second == room number

					for (int r = 0; r < nb_locations; ++r) // initialize unassigned rooms
					{
						if (current_solution.at(timeslot1, r) >= 0)
							unassigned_nodes.push_back(std::pair<int, int>(timeslot1, r));
						if (current_solution.at(timeslot2, r) >= 0)
							unassigned_nodes.push_back(std::pair<int, int>(timeslot2, r));
					}

					std::vector<std::vector<std::pair<int, int>>> kempe_chains;
					while (true)
					{
						if (unassigned_nodes.size() <= 0)
							break;

						std::vector<std::pair<int, int>> kempe_chain_current;
						kempe_chain_current.push_back(unassigned_nodes.back());
						unassigned_nodes.pop_back();

						for (int i = 0; i < kempe_chain_current.size(); ++i)
						{
							for (int j = unassigned_nodes.size() - 1; j >= 0; --j)
							{
								if (kempe_chain_current.at(i).first != unassigned_nodes.at(j).first) // different timeslots
								{
									if (kempe_chain_current.at(i).second == unassigned_nodes.at(j).second	// same room
										|| get_sessionsessionconflict(current_solution.at(kempe_chain_current.at(i).first, kempe_chain_current.at(i).second),
											current_solution.at(unassigned_nodes.at(j).first, unassigned_nodes.at(j).second))) // lecture hase conflict
									{
										kempe_chain_current.push_back(unassigned_nodes.at(j));
										unassigned_nodes.erase(unassigned_nodes.begin() + j);
									}
								}
							}
						}

						// sort chain based on room
						std::sort(kempe_chain_current.begin(), kempe_chain_current.end(), sort_on_room);

						// save chain
						kempe_chains.push_back(kempe_chain_current);
					}



					// (3) for every chain, calculate objective value if the lectures in the chain are swapped between rooms
					double best_obj_val_KC = 1e20;
					int best_index_KC = -1;

					for (int i = 0; i < kempe_chains.size(); ++i)
					{
						// calculate candidate objective value
						double candidate_objective = evaluate_incremental(current_solution, current_objective.objective_value, kempe_chains.at(i), timeslot1, timeslot2);

						// save if improvement
						if (candidate_objective < best_obj_val_KC) // best chain of all chains
						{
							best_obj_val_KC = candidate_objective;

							double percentage_deterioration = (candidate_objective - current_objective.objective_value) / current_objective.objective_value;
							if (candidate_objective < current_objective.objective_value
								|| std::exp(-percentage_deterioration / heuristic_master::_SA_temperature) < prob_accept)
							{
								best_index_KC = i;
							}
						}
					}


					// (4) implement best chain if improvement
					if (best_index_KC != -1)
					{
						// recalculate so that correct values are in the new vectors
						double candidate_objective = evaluate_incremental(current_solution, current_objective.objective_value, kempe_chains.at(best_index_KC), timeslot1, timeslot2);

						// implement the changes
						for (int ii = 0; ii < kempe_chains.at(best_index_KC).size(); ++ii)
						{
							if (ii < kempe_chains.at(best_index_KC).size() - 1)
							{
								if (kempe_chains.at(best_index_KC).at(ii).second == kempe_chains.at(best_index_KC).at(ii + 1).second) // two lectures same room different timeslot (by definition timeslot should be different)
								{
									// swap two lectures
									int lec_temp = current_solution.at(kempe_chains.at(best_index_KC).at(ii).first, kempe_chains.at(best_index_KC).at(ii).second);
									current_solution.at(kempe_chains.at(best_index_KC).at(ii).first, kempe_chains.at(best_index_KC).at(ii).second) = current_solution.at(kempe_chains.at(best_index_KC).at(ii + 1).first, kempe_chains.at(best_index_KC).at(ii + 1).second);
									current_solution.at(kempe_chains.at(best_index_KC).at(ii + 1).first, kempe_chains.at(best_index_KC).at(ii + 1).second) = lec_temp;

									// two lectures swapped, so skip ii+1
									++ii;
								}
								else
								{
									// move lecture in current room to lecture in other room
									if (kempe_chains.at(best_index_KC).at(ii).first == timeslot1)
									{
										int lec_temp = current_solution.at(timeslot1, kempe_chains.at(best_index_KC).at(ii).second);
										current_solution.at(timeslot1, kempe_chains.at(best_index_KC).at(ii).second) = -1;
										current_solution.at(timeslot2, kempe_chains.at(best_index_KC).at(ii).second) = lec_temp;
									}
									else
									{
										int lec_temp = current_solution.at(timeslot2, kempe_chains.at(best_index_KC).at(ii).second);
										current_solution.at(timeslot2, kempe_chains.at(best_index_KC).at(ii).second) = -1;
										current_solution.at(timeslot1, kempe_chains.at(best_index_KC).at(ii).second) = lec_temp;
									}
								}
							}
							else
							{
								// move lecture in current room to lecture in other room
								if (kempe_chains.at(best_index_KC).at(ii).first == timeslot1)
								{
									int lec_temp = current_solution.at(timeslot1, kempe_chains.at(best_index_KC).at(ii).second);
									current_solution.at(timeslot1, kempe_chains.at(best_index_KC).at(ii).second) = -1;
									current_solution.at(timeslot2, kempe_chains.at(best_index_KC).at(ii).second) = lec_temp;
								}
								else
								{
									int lec_temp = current_solution.at(timeslot2, kempe_chains.at(best_index_KC).at(ii).second);
									current_solution.at(timeslot2, kempe_chains.at(best_index_KC).at(ii).second) = -1;
									current_solution.at(timeslot1, kempe_chains.at(best_index_KC).at(ii).second) = lec_temp;
								}
							}
						}

						// save the objective value
						current_objective.objective_value = best_obj_val_KC;

						// update the values for individual objectives and constraints
						current_objective.constraint_violations_scheduling_conflicts = new_constraint_violations_scheduling_conflicts;
						current_objective.constraint_violations_correct_room = new_constraint_violations_correct_room;
						current_objective.constraint_violations_teacher_working_time = new_constraint_violations_teacher_working_time;
						current_objective.constraint_violations_compactness_constraints = new_constraint_violations_compactness_constraints;
						current_objective.preference_score = new_preference_score;

						// update the evacuation/travel times
						{

							{
								current_objective.evacuations_timeslot_max_travel_time.at(timeslot1) = new_max_evac_time_changed_ts.at(0);
								current_objective.evacuations_timeslot_max_travel_time.at(timeslot2) = new_max_evac_time_changed_ts.at(1);
							}
						}


						// check if better than best solution
						if (current_objective.objective_value < best_objective.objective_value)
						{
							best_objective = current_objective;
							best_solution = current_solution;
							std::string text = "\n\nNew best solution found (Kempe Chain) in thread " + std::to_string(_process_id) + "!\nObjective value " + std::to_string(current_objective.objective_value);
							std::cout << text;

							//best_solution.print();
						}
					}
				}
			}

		}
	}







	double heuristic_subprocess::evaluate_incremental(matrix2D<int>& solution, int timeslot, int room1, int room2)
	{
		// 1. start from current solution
		new_constraint_violations_scheduling_conflicts = current_objective.constraint_violations_scheduling_conflicts;
		new_constraint_violations_correct_room = current_objective.constraint_violations_correct_room;
		new_constraint_violations_teacher_working_time = current_objective.constraint_violations_teacher_working_time;
		new_constraint_violations_compactness_constraints = current_objective.constraint_violations_compactness_constraints;
		new_preference_score = current_objective.preference_score;

		// 2. Deduct old values
		{
			// A. TIMETABLE 
			// I. CONSTRAINTS
			// 1. No conflicts: does not change

			// 2. Correct rooms
			{
				int violations = 0;

				if (solution.at(timeslot, room1) >= 0 && !get_sessionlocationpossible(solution.at(timeslot, room1), room1)) // before change to solution
					++violations;
				if (solution.at(timeslot, room2) >= 0 && !get_sessionlocationpossible(solution.at(timeslot, room2), room2))
					++violations;

				new_constraint_violations_correct_room -= violations;
			}

			// 3. Teacher working time regulations: does not change

			// 4. Compactness constraints: does not change


			// II. OBJECTIVE (PREFERENCES)
			// does not change
		}

		// 3. Implement changes
		int lec_temp = solution.at(timeslot, room1);
		solution.at(timeslot, room1) = solution.at(timeslot, room2);
		solution.at(timeslot, room2) = lec_temp;

		// 4. Add new values
		{
			// A. TIMETABLE 
			// I. CONSTRAINTS
			// 1. No conflicts: does not change

			// 2. Correct rooms
			{
				int violations = 0;

				if (solution.at(timeslot, room1) >= 0 && !get_sessionlocationpossible(solution.at(timeslot, room1), room1)) // after change to solution
					++violations;
				if (solution.at(timeslot, room2) >= 0 && !get_sessionlocationpossible(solution.at(timeslot, room2), room2))
					++violations;

				new_constraint_violations_correct_room += violations;
			}

			// 3. Teacher working time regulations: does not change

			// 4. Compactness constraints: does not change


			// II. OBJECTIVE (PREFERENCES)
			// does not change
		}



		// 5. calculate flows for relevant timeslots
		// B. FLOWS
		// I. Evacuations
		double objective_evacuations = 0.0;
		{
			current_objective.evacuations_timeslot_lecture_uses_path.fill(0.0);
			current_objective.evacuations_timeslot_flow_per_arc.fill(0.0);
			current_objective.evacuations_timeslot_time_per_arc.fill(0.0);
			current_objective.evacuations_timeslot_lecture_travel_time.fill(0.0);
			new_max_evac_time_changed_ts_roomswap = 0.0;

			{
				// 1. who uses which paths
				for (int r = 0; r < nb_locations; ++r)
				{
					int lec = solution.at(timeslot, r);
					if (lec >= 0)
					{
						for (int p = 0; p < nb_paths; ++p)
							current_objective.evacuations_timeslot_lecture_uses_path.at(timeslot, lec, p) = get_roomroompath(r, nb_locations, p);
					}
				}

				// 2. flow per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					for (int l = 0; l < nb_sessions; ++l)
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							double nbp = current_objective.evacuations_timeslot_lecture_uses_path.at(timeslot, l, p) * session_nb_people.at(l);
							if (get_patharc(p, ij))
								current_objective.evacuations_timeslot_flow_per_arc.at(timeslot, ij) += nbp;
						}
					}
				}

				// 3. time per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					current_objective.evacuations_timeslot_time_per_arc.at(timeslot, ij) = arc_length.at(ij) / walking_alpha * current_objective.evacuations_timeslot_flow_per_arc.at(timeslot, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
					if (arc_stairs.at(ij))
						current_objective.evacuations_timeslot_time_per_arc.at(timeslot, ij) *= speed_correction_stairs;
				}

				// 4. time per lecture in total
				for (int l = 0; l < nb_sessions; ++l)
				{
					double time_longest_path = 0.0;
					for (int p = 0; p < nb_paths; ++p)
					{
						if (current_objective.evacuations_timeslot_lecture_uses_path.at(timeslot, l, p) > 0.01)
						{
							double time_current_path = 0.0;
							for (int ij = 0; ij < nb_arcs; ++ij)
							{
								if (get_patharc(p, ij))
									time_current_path += current_objective.evacuations_timeslot_time_per_arc.at(timeslot, ij);
							}
							if (time_current_path > time_longest_path)
								time_longest_path = time_current_path;
						}
					}
					current_objective.evacuations_timeslot_lecture_travel_time.at(timeslot, l) = time_longest_path;
				}

				// 5. maximum evacuation time per timeslot
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (current_objective.evacuations_timeslot_lecture_travel_time.at(timeslot, l) > new_max_evac_time_changed_ts_roomswap)
						new_max_evac_time_changed_ts_roomswap = current_objective.evacuations_timeslot_lecture_travel_time.at(timeslot, l);
				}
			}

			// 6. objective evacuations
			if (heuristic_master::_objective_type == heuristic_master::objective_type::maximum_over_timeslots)
			{
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (t == timeslot)
					{
						if (new_max_evac_time_changed_ts_roomswap > objective_evacuations)
							objective_evacuations = new_max_evac_time_changed_ts_roomswap;
					}
					else
					{
						if (current_objective.evacuations_timeslot_max_travel_time.at(t) > objective_evacuations)
							objective_evacuations = current_objective.evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (t == timeslot)
					{
						objective_evacuations += new_max_evac_time_changed_ts_roomswap;
					}
					else
					{
						objective_evacuations += current_objective.evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
		}



		// 6. Undo changes
		lec_temp = solution.at(timeslot, room1);
		solution.at(timeslot, room1) = solution.at(timeslot, room2);
		solution.at(timeslot, room2) = lec_temp;


		// 7. recalculate objective value
		return (heuristic_master::_penalty_value_constraint_violation * (new_constraint_violations_scheduling_conflicts
			+ new_constraint_violations_correct_room
			+ new_constraint_violations_teacher_working_time
			+ new_constraint_violations_compactness_constraints)
			+ heuristic_master::_lambda * new_preference_score + (1 - heuristic_master::_lambda) * objective_evacuations);
	}




	double heuristic_subprocess::evaluate_incremental(matrix2D<int>& solution, int timeslot1, int timeslot2, int room1, int room2)
	{
		// 1. start from current solution
		new_constraint_violations_scheduling_conflicts = current_objective.constraint_violations_scheduling_conflicts;
		new_constraint_violations_correct_room = current_objective.constraint_violations_correct_room;
		new_constraint_violations_teacher_working_time = current_objective.constraint_violations_teacher_working_time;
		new_constraint_violations_compactness_constraints = current_objective.constraint_violations_compactness_constraints;
		new_preference_score = current_objective.preference_score;

		// 2. Deduct old values
		{
			// A. TIMETABLE 
			// I. CONSTRAINTS
			// 1. No conflicts
			{
				int violations = 0;
				for (int r = 0; r < nb_locations; ++r)
				{
					if (r != room1 && solution.at(timeslot1, room1) >= 0 && solution.at(timeslot1, r) >= 0 && get_sessionsessionconflict(solution.at(timeslot1, room1), solution.at(timeslot1, r)))
						++violations;
					if (r != room2 && solution.at(timeslot2, room2) >= 0 && solution.at(timeslot2, r) >= 0 && get_sessionsessionconflict(solution.at(timeslot2, room2), solution.at(timeslot2, r)))
						++violations;
				}
				new_constraint_violations_scheduling_conflicts -= violations;

			}

			// 2. Correct rooms
			{
				int violations = 0;

				if (solution.at(timeslot1, room1) >= 0 && !get_sessionlocationpossible(solution.at(timeslot1, room1), room1)) // before change to solution
					++violations;
				if (solution.at(timeslot2, room2) >= 0 && !get_sessionlocationpossible(solution.at(timeslot2, room2), room2))
					++violations;

				new_constraint_violations_correct_room -= violations;
			}

			// 3.A teacher working time regulations: at most 4 lectures per day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					// only days that change
					if (timeslot1 / nb_timeslots_per_day == day || timeslot2 / nb_timeslots_per_day == day)
					{
						int first_timeslot = day * nb_timeslots_per_day;
						int last_timeslot = (day + 1) * nb_timeslots_per_day;

						int teaches = 0;
						for (int t = first_timeslot; t < last_timeslot; ++t)
						{
							for (int r = 0; r < nb_locations; ++r)
							{
								int lec = solution.at(t, r);
								if (lec >= 0 && get_teachersession(d, lec))
									++teaches;
							}
						}
						if (teaches > 4)
							--new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 3.B teacher working time regulations: no more than 3 lectures consecutively
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					int first_timeslot = day * nb_timeslots_per_day;
					int last_timeslot = first_timeslot + nb_timeslots_per_day - 3;

					int teaches = 0;
					for (int t = first_timeslot; t < last_timeslot; ++t)
					{
						// only if changed timeslots fall in range
						if ((timeslot1 >= t && timeslot1 <= t + 3) || (timeslot2 >= t && timeslot2 <= t + 3))
						{
							bool teaches_ts1 = false, teaches_ts2 = false, teaches_ts3 = false, teaches_ts4 = false;
							for (int r = 0; r < nb_locations; ++r)
							{
								int lec1 = solution.at(t, r);
								int lec2 = solution.at(t + 1, r);
								int lec3 = solution.at(t + 2, r);
								int lec4 = solution.at(t + 3, r);

								if (lec1 >= 0 && get_teachersession(d, lec1))
									++teaches_ts1 = true;
								if (lec2 >= 0 && get_teachersession(d, lec2))
									++teaches_ts2 = true;
								if (lec3 >= 0 && get_teachersession(d, lec3))
									++teaches_ts3 = true;
								if (lec4 >= 0 && get_teachersession(d, lec4))
									++teaches_ts4 = true;
							}
							if (teaches_ts1 && teaches_ts2 && teaches_ts3 && teaches_ts4)
								--new_constraint_violations_teacher_working_time;
						}
					}
				}
			}

			// 3.C teacher working time regulations: not last timeslot of previous day and first timeslot of next day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 1; day < nb_days; ++day)
				{
					int first_timeslot_of_next_day = day * nb_timeslots_per_day;
					int last_timeslot_of_previous_day = day * nb_timeslots_per_day - 1;

					// only if changed timeslots equal one of the two timeslots
					if ((timeslot1 == first_timeslot_of_next_day || timeslot1 == last_timeslot_of_previous_day)
						|| (timeslot2 == first_timeslot_of_next_day || timeslot2 == last_timeslot_of_previous_day))
					{
						bool teaches_t1 = false, teaches_t2 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							int lec1 = solution.at(last_timeslot_of_previous_day, r);
							if (lec1 >= 0 && get_teachersession(d, lec1))
								teaches_t1 = true;

							int lec2 = solution.at(first_timeslot_of_next_day, r);
							if (lec2 >= 0 && get_teachersession(d, lec2))
								teaches_t2 = true;
						}
						if (teaches_t1 && teaches_t2)
							--new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 3.D teacher working time regulations: not first and last timeslot of same day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					int first_timeslot_of_day = day * nb_timeslots_per_day;
					int last_timeslot_of_day = (day + 1) * nb_timeslots_per_day - 1;

					// only if changed timeslots equal one of the two timeslots
					if ((timeslot1 == first_timeslot_of_day || timeslot1 == last_timeslot_of_day)
						|| (timeslot2 == first_timeslot_of_day || timeslot2 == last_timeslot_of_day))
					{
						bool teaches_t1 = false, teaches_t2 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							int lec1 = solution.at(last_timeslot_of_day, r);
							if (lec1 >= 0 && get_teachersession(d, lec1))
								teaches_t1 = true;

							int lec2 = solution.at(first_timeslot_of_day, r);
							if (lec2 >= 0 && get_teachersession(d, lec2))
								teaches_t2 = true;
						}
						if (teaches_t1 && teaches_t2)
							--new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 4. Compactness constraints
			if (nb_timeslots_per_day == 5 || nb_timeslots_per_day == 6)
			{
				for (int s = 0; s < nb_series; ++s)
				{
					for (int day = 0; day < nb_days; ++day)
					{
						for (int t = 0; t < 3; ++t) // (1-3, 2-4,) 3-5
						{
							int ts1 = day * nb_timeslots_per_day + t;
							int ts2 = day * nb_timeslots_per_day + t + 1;
							int ts3 = day * nb_timeslots_per_day + t + 2;

							if (timeslot1 == ts1 || timeslot1 == ts2 || timeslot1 == ts3
								|| timeslot2 == ts1 || timeslot2 == ts2 || timeslot2 == ts3)
							{
								bool classts1 = false, classts2 = false, classts3 = false;
								for (int r = 0; r < nb_locations; ++r)
								{
									if (solution.at(ts1, r) >= 0 && get_seriessession(s, solution.at(ts1, r)))
										classts1 = true;
									if (solution.at(ts2, r) >= 0 && get_seriessession(s, solution.at(ts2, r)))
										classts2 = true;
									if (solution.at(ts3, r) >= 0 && get_seriessession(s, solution.at(ts3, r)))
										classts3 = true;
								}

								if (classts1 && !classts2 && classts3)
									--new_constraint_violations_compactness_constraints;
							}
						}
					}
				}
			}
			else
			{
				// N/A 
				current_objective.constraint_violations_compactness_constraints = 0;
			}

			// II. OBJECTIVE (PREFERENCES)
			{
				int lec = solution.at(timeslot1, room1);
				if (lec >= 0)
					new_preference_score -= (get_costsessiontimeslot(lec, timeslot1) + 1000 * get_costsession_ts_educational(lec, timeslot1));
				lec = solution.at(timeslot2, room2);
				if (lec >= 0)
					new_preference_score -= (get_costsessiontimeslot(lec, timeslot2) + 1000 * get_costsession_ts_educational(lec, timeslot2));
			}
		}

		// 3. Implement changes
		int lec_temp = solution.at(timeslot1, room1);
		solution.at(timeslot1, room1) = solution.at(timeslot2, room2);
		solution.at(timeslot2, room2) = lec_temp;

		// 4. Add new values
		{
			// A. TIMETABLE 
			// I. CONSTRAINTS
			// 1. No conflicts
			{
				int violations = 0;
				for (int r = 0; r < nb_locations; ++r)
				{
					if (r != room1 && solution.at(timeslot1, room1) >= 0 && solution.at(timeslot1, r) >= 0 && get_sessionsessionconflict(solution.at(timeslot1, room1), solution.at(timeslot1, r)))
						++violations;
					if (r != room2 && solution.at(timeslot2, room2) >= 0 && solution.at(timeslot2, r) >= 0 && get_sessionsessionconflict(solution.at(timeslot2, room2), solution.at(timeslot2, r)))
						++violations;
				}
				new_constraint_violations_scheduling_conflicts += violations;

			}

			// 2. Correct rooms
			{
				int violations = 0;

				if (solution.at(timeslot1, room1) >= 0 && !get_sessionlocationpossible(solution.at(timeslot1, room1), room1)) // after change to solution
					++violations;
				if (solution.at(timeslot2, room2) >= 0 && !get_sessionlocationpossible(solution.at(timeslot2, room2), room2))
					++violations;

				new_constraint_violations_correct_room += violations;
			}

			// 3. Teacher working time regulations
			// 3.A teacher working time regulations: at most 4 lectures per day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					// only days that change
					if (timeslot1 / nb_timeslots_per_day == day || timeslot2 / nb_timeslots_per_day == day)
					{
						int first_timeslot = day * nb_timeslots_per_day;
						int last_timeslot = (day + 1) * nb_timeslots_per_day;

						int teaches = 0;
						for (int t = first_timeslot; t < last_timeslot; ++t)
						{
							for (int r = 0; r < nb_locations; ++r)
							{
								int lec = solution.at(t, r);
								if (lec >= 0 && get_teachersession(d, lec))
									++teaches;
							}
						}
						if (teaches > 4)
							++new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 3.B teacher working time regulations: no more than 3 lectures consecutively
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					int first_timeslot = day * nb_timeslots_per_day;
					int last_timeslot = first_timeslot + nb_timeslots_per_day - 3;

					int teaches = 0;
					for (int t = first_timeslot; t < last_timeslot; ++t)
					{
						// only if changed timeslots fall in range
						if ((timeslot1 >= t && timeslot1 <= t + 3) || (timeslot2 >= t && timeslot2 <= t + 3))
						{
							bool teaches_ts1 = false, teaches_ts2 = false, teaches_ts3 = false, teaches_ts4 = false;
							for (int r = 0; r < nb_locations; ++r)
							{
								int lec1 = solution.at(t, r);
								int lec2 = solution.at(t + 1, r);
								int lec3 = solution.at(t + 2, r);
								int lec4 = solution.at(t + 3, r);

								if (lec1 >= 0 && get_teachersession(d, lec1))
									++teaches_ts1 = true;
								if (lec2 >= 0 && get_teachersession(d, lec2))
									++teaches_ts2 = true;
								if (lec3 >= 0 && get_teachersession(d, lec3))
									++teaches_ts3 = true;
								if (lec4 >= 0 && get_teachersession(d, lec4))
									++teaches_ts4 = true;
							}
							if (teaches_ts1 && teaches_ts2 && teaches_ts3 && teaches_ts4)
								++new_constraint_violations_teacher_working_time;
						}
					}
				}
			}

			// 3.C teacher working time regulations: not last timeslot of previous day and first timeslot of next day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 1; day < nb_days; ++day)
				{
					int first_timeslot_of_next_day = day * nb_timeslots_per_day;
					int last_timeslot_of_previous_day = day * nb_timeslots_per_day - 1;

					// only if changed timeslots equal one of the two timeslots
					if ((timeslot1 == first_timeslot_of_next_day || timeslot1 == last_timeslot_of_previous_day)
						|| (timeslot2 == first_timeslot_of_next_day || timeslot2 == last_timeslot_of_previous_day))
					{
						bool teaches_t1 = false, teaches_t2 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							int lec1 = solution.at(last_timeslot_of_previous_day, r);
							if (lec1 >= 0 && get_teachersession(d, lec1))
								teaches_t1 = true;

							int lec2 = solution.at(first_timeslot_of_next_day, r);
							if (lec2 >= 0 && get_teachersession(d, lec2))
								teaches_t2 = true;
						}
						if (teaches_t1 && teaches_t2)
							++new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 3.D teacher working time regulations: not first and last timeslot of same day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					int first_timeslot_of_day = day * nb_timeslots_per_day;
					int last_timeslot_of_day = (day + 1) * nb_timeslots_per_day - 1;

					// only if changed timeslots equal one of the two timeslots
					if ((timeslot1 == first_timeslot_of_day || timeslot1 == last_timeslot_of_day)
						|| (timeslot2 == first_timeslot_of_day || timeslot2 == last_timeslot_of_day))
					{
						bool teaches_t1 = false, teaches_t2 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							int lec1 = solution.at(last_timeslot_of_day, r);
							if (lec1 >= 0 && get_teachersession(d, lec1))
								teaches_t1 = true;

							int lec2 = solution.at(first_timeslot_of_day, r);
							if (lec2 >= 0 && get_teachersession(d, lec2))
								teaches_t2 = true;
						}
						if (teaches_t1 && teaches_t2)
							++new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 4. Compactness constraints
			if (nb_timeslots_per_day == 5 || nb_timeslots_per_day == 6)
			{
				for (int s = 0; s < nb_series; ++s)
				{
					for (int day = 0; day < nb_days; ++day)
					{
						for (int t = 0; t < 3; ++t) // (1-3, 2-4,) 3-5
						{
							int ts1 = day * nb_timeslots_per_day + t;
							int ts2 = day * nb_timeslots_per_day + t + 1;
							int ts3 = day * nb_timeslots_per_day + t + 2;

							if (timeslot1 == ts1 || timeslot1 == ts2 || timeslot1 == ts3
								|| timeslot2 == ts1 || timeslot2 == ts2 || timeslot2 == ts3)
							{
								bool classts1 = false, classts2 = false, classts3 = false;
								for (int r = 0; r < nb_locations; ++r)
								{
									if (solution.at(ts1, r) >= 0 && get_seriessession(s, solution.at(ts1, r)))
										classts1 = true;
									if (solution.at(ts2, r) >= 0 && get_seriessession(s, solution.at(ts2, r)))
										classts2 = true;
									if (solution.at(ts3, r) >= 0 && get_seriessession(s, solution.at(ts3, r)))
										classts3 = true;
								}

								if (classts1 && !classts2 && classts3)
									++new_constraint_violations_compactness_constraints;
							}
						}
					}
				}
			}
			else
			{
				// N/A 
				current_objective.constraint_violations_compactness_constraints = 0;
			}

			// II. OBJECTIVE (PREFERENCES)
			{
				int lec = solution.at(timeslot1, room1);
				if (lec >= 0)
					new_preference_score += (get_costsessiontimeslot(lec, timeslot1) + 1000 * get_costsession_ts_educational(lec, timeslot1));
				lec = solution.at(timeslot2, room2);
				if (lec >= 0)
					new_preference_score += (get_costsessiontimeslot(lec, timeslot2) + 1000 * get_costsession_ts_educational(lec, timeslot2));
			}
			if (heuristic_master::_constraint_preferences && new_preference_score > heuristic_master::_constraint_preferences_value)
				new_preference_score += heuristic_master::_penalty_value_constraint_violation;

		}



		// 5. calculate flows for relevant timeslots
		// B. FLOWS
		// I. Evacuations
		double objective_evacuations = 0.0;
		{
			current_objective.evacuations_timeslot_lecture_uses_path.fill(0.0);
			current_objective.evacuations_timeslot_flow_per_arc.fill(0.0);
			current_objective.evacuations_timeslot_time_per_arc.fill(0.0);
			current_objective.evacuations_timeslot_lecture_travel_time.fill(0.0);
			new_max_evac_time_changed_ts.at(0) = 0;
			new_max_evac_time_changed_ts.at(1) = 0;

			int index_ts = 0;
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t == timeslot1 || t == timeslot2)
				{
					// 1. who uses which paths
					for (int r = 0; r < nb_locations; ++r)
					{
						int lec = solution.at(t, r);
						if (lec >= 0)
						{
							for (int p = 0; p < nb_paths; ++p)
								current_objective.evacuations_timeslot_lecture_uses_path.at(t, lec, p) = get_roomroompath(r, nb_locations, p);
						}
					}

					// 2. flow per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						for (int l = 0; l < nb_sessions; ++l)
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								double nbp = current_objective.evacuations_timeslot_lecture_uses_path.at(t, l, p) * session_nb_people.at(l);
								if (get_patharc(p, ij))
									current_objective.evacuations_timeslot_flow_per_arc.at(t, ij) += nbp;
							}
						}
					}

					// 3. time per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						current_objective.evacuations_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * current_objective.evacuations_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
						if (arc_stairs.at(ij))
							current_objective.evacuations_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
					}

					// 4. time per lecture in total
					for (int l = 0; l < nb_sessions; ++l)
					{
						double time_longest_path = 0.0;
						for (int p = 0; p < nb_paths; ++p)
						{
							if (current_objective.evacuations_timeslot_lecture_uses_path.at(t, l, p) > 0.01)
							{
								double time_current_path = 0.0;
								for (int ij = 0; ij < nb_arcs; ++ij)
								{
									if (get_patharc(p, ij))
										time_current_path += current_objective.evacuations_timeslot_time_per_arc.at(t, ij);
								}
								if (time_current_path > time_longest_path)
									time_longest_path = time_current_path;
							}
						}
						current_objective.evacuations_timeslot_lecture_travel_time.at(t, l) = time_longest_path;
					}

					// 5. maximum evacuation time per timeslot
					for (int l = 0; l < nb_sessions; ++l)
					{
						if (current_objective.evacuations_timeslot_lecture_travel_time.at(t, l) > new_max_evac_time_changed_ts.at(index_ts))
							new_max_evac_time_changed_ts.at(index_ts) = current_objective.evacuations_timeslot_lecture_travel_time.at(t, l);
					}

					++index_ts; // go to next timeslot
				}
			}

			// 6. objective evacuations
			if (heuristic_master::_objective_type == heuristic_master::objective_type::maximum_over_timeslots)
			{
				index_ts = 0;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (t == timeslot1 || t == timeslot2)
					{
						if (new_max_evac_time_changed_ts.at(index_ts) > objective_evacuations)
							objective_evacuations = new_max_evac_time_changed_ts.at(index_ts);
						++index_ts;
					}
					else
					{
						if (current_objective.evacuations_timeslot_max_travel_time.at(t) > objective_evacuations)
							objective_evacuations = current_objective.evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				index_ts = 0;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (t == timeslot1 || t == timeslot2)
					{
						objective_evacuations += new_max_evac_time_changed_ts.at(index_ts);
						++index_ts;
					}
					else
					{
						objective_evacuations += current_objective.evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
		}



		// 6. Undo changes
		lec_temp = solution.at(timeslot1, room1);
		solution.at(timeslot1, room1) = solution.at(timeslot2, room2);
		solution.at(timeslot2, room2) = lec_temp;


		// 7. recalculate objective value
		return (heuristic_master::_penalty_value_constraint_violation * (new_constraint_violations_scheduling_conflicts
			+ new_constraint_violations_correct_room
			+ new_constraint_violations_teacher_working_time
			+ new_constraint_violations_compactness_constraints)
			+ heuristic_master::_lambda * new_preference_score + (1 - heuristic_master::_lambda) * objective_evacuations);
	}




	double heuristic_subprocess::evaluate_incremental(matrix2D<int>& solution, double current_obj, const std::vector<std::pair<int, int>>& kempe_chain, int timeslot1, int timeslot2)
	{
		// no empty chain
		if (kempe_chain.empty())
			return current_obj;

		// 1. start from current solution
		new_constraint_violations_scheduling_conflicts = current_objective.constraint_violations_scheduling_conflicts;
		new_constraint_violations_correct_room = current_objective.constraint_violations_correct_room;
		new_constraint_violations_teacher_working_time = current_objective.constraint_violations_teacher_working_time;
		new_constraint_violations_compactness_constraints = current_objective.constraint_violations_compactness_constraints;
		new_preference_score = current_objective.preference_score;

		// 2. Deduct old values
		{
			// A. TIMETABLE 
			// I. CONSTRAINTS
			// 1. No conflicts
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t == timeslot1 || t == timeslot2)
				{
					int violations = 0;
					for (int r1 = 0; r1 < nb_locations; ++r1)
					{
						for (int r2 = r1 + 1; r2 < nb_locations; ++r2)
						{
							if (solution.at(t, r1) >= 0 && solution.at(t, r2) >= 0 && get_sessionsessionconflict(solution.at(t, r1), solution.at(t, r2)))
								++violations;
						}
					}
					new_constraint_violations_scheduling_conflicts -= violations;
				}
			}

			// 2. Correct rooms
			{
				int violations = 0;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (t == timeslot1 || t == timeslot2)
					{
						for (int r = 0; r < nb_locations; ++r)
						{

							if (solution.at(t, r) >= 0 && !get_sessionlocationpossible(solution.at(t, r), r)) // before change to solution
								++violations;

						}
					}
				}
				new_constraint_violations_correct_room -= violations;
			}

			// 3.A teacher working time regulations: at most 4 lectures per day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					// only days that change
					if (timeslot1 / nb_timeslots_per_day == day || timeslot2 / nb_timeslots_per_day == day)
					{
						int first_timeslot = day * nb_timeslots_per_day;
						int last_timeslot = (day + 1) * nb_timeslots_per_day;

						int teaches = 0;
						for (int t = first_timeslot; t < last_timeslot; ++t)
						{
							for (int r = 0; r < nb_locations; ++r)
							{
								int lec = solution.at(t, r);
								if (lec >= 0 && get_teachersession(d, lec))
									++teaches;
							}
						}
						if (teaches > 4)
							--new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 3.B teacher working time regulations: no more than 3 lectures consecutively
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					int first_timeslot = day * nb_timeslots_per_day;
					int last_timeslot = first_timeslot + nb_timeslots_per_day - 3;

					int teaches = 0;
					for (int t = first_timeslot; t < last_timeslot; ++t)
					{
						// only if changed timeslots fall in range
						if ((timeslot1 >= t && timeslot1 <= t + 3) || (timeslot2 >= t && timeslot2 <= t + 3))
						{
							bool teaches_ts1 = false, teaches_ts2 = false, teaches_ts3 = false, teaches_ts4 = false;
							for (int r = 0; r < nb_locations; ++r)
							{
								int lec1 = solution.at(t, r);
								int lec2 = solution.at(t + 1, r);
								int lec3 = solution.at(t + 2, r);
								int lec4 = solution.at(t + 3, r);

								if (lec1 >= 0 && get_teachersession(d, lec1))
									++teaches_ts1 = true;
								if (lec2 >= 0 && get_teachersession(d, lec2))
									++teaches_ts2 = true;
								if (lec3 >= 0 && get_teachersession(d, lec3))
									++teaches_ts3 = true;
								if (lec4 >= 0 && get_teachersession(d, lec4))
									++teaches_ts4 = true;
							}
							if (teaches_ts1 && teaches_ts2 && teaches_ts3 && teaches_ts4)
								--new_constraint_violations_teacher_working_time;
						}
					}
				}
			}

			// 3.C teacher working time regulations: not last timeslot of previous day and first timeslot of next day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 1; day < nb_days; ++day)
				{
					int first_timeslot_of_next_day = day * nb_timeslots_per_day;
					int last_timeslot_of_previous_day = day * nb_timeslots_per_day - 1;

					// only if changed timeslots equal one of the two timeslots
					if ((timeslot1 == first_timeslot_of_next_day || timeslot1 == last_timeslot_of_previous_day)
						|| (timeslot2 == first_timeslot_of_next_day || timeslot2 == last_timeslot_of_previous_day))
					{
						bool teaches_t1 = false, teaches_t2 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							int lec1 = solution.at(last_timeslot_of_previous_day, r);
							if (lec1 >= 0 && get_teachersession(d, lec1))
								teaches_t1 = true;

							int lec2 = solution.at(first_timeslot_of_next_day, r);
							if (lec2 >= 0 && get_teachersession(d, lec2))
								teaches_t2 = true;
						}
						if (teaches_t1 && teaches_t2)
							--new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 3.D teacher working time regulations: not first and last timeslot of same day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					int first_timeslot_of_day = day * nb_timeslots_per_day;
					int last_timeslot_of_day = (day + 1) * nb_timeslots_per_day - 1;

					// only if changed timeslots equal one of the two timeslots
					if ((timeslot1 == first_timeslot_of_day || timeslot1 == last_timeslot_of_day)
						|| (timeslot2 == first_timeslot_of_day || timeslot2 == last_timeslot_of_day))
					{
						bool teaches_t1 = false, teaches_t2 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							int lec1 = solution.at(last_timeslot_of_day, r);
							if (lec1 >= 0 && get_teachersession(d, lec1))
								teaches_t1 = true;

							int lec2 = solution.at(first_timeslot_of_day, r);
							if (lec2 >= 0 && get_teachersession(d, lec2))
								teaches_t2 = true;
						}
						if (teaches_t1 && teaches_t2)
							--new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 4. Compactness constraints: does not change
			if (nb_timeslots_per_day == 5 || nb_timeslots_per_day == 6)
			{
				for (int s = 0; s < nb_series; ++s)
				{
					for (int day = 0; day < nb_days; ++day)
					{
						for (int t = 0; t < 3; ++t) // (1-3, 2-4,) 3-5
						{
							int ts1 = day * nb_timeslots_per_day + t;
							int ts2 = day * nb_timeslots_per_day + t + 1;
							int ts3 = day * nb_timeslots_per_day + t + 2;

							if (timeslot1 == ts1 || timeslot1 == ts2 || timeslot1 == ts3
								|| timeslot2 == ts1 || timeslot2 == ts2 || timeslot2 == ts3)
							{
								bool classts1 = false, classts2 = false, classts3 = false;
								for (int r = 0; r < nb_locations; ++r)
								{
									if (solution.at(ts1, r) >= 0 && get_seriessession(s, solution.at(ts1, r)))
										classts1 = true;
									if (solution.at(ts2, r) >= 0 && get_seriessession(s, solution.at(ts2, r)))
										classts2 = true;
									if (solution.at(ts3, r) >= 0 && get_seriessession(s, solution.at(ts3, r)))
										classts3 = true;
								}

								if (classts1 && !classts2 && classts3)
									--new_constraint_violations_compactness_constraints;
							}
						}
					}
				}
			}
			else
			{
				// N/A 
				current_objective.constraint_violations_compactness_constraints = 0;
			}

			// II. OBJECTIVE (PREFERENCES)
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t == timeslot1 || t == timeslot2)
				{
					for (int r = 0; r < nb_locations; ++r)
					{
						int lec = solution.at(t, r);
						if (lec >= 0)
							new_preference_score -= (get_costsessiontimeslot(lec, t) + 1000 * get_costsession_ts_educational(lec, t));
					}
				}
			}
		}


		// 3. Implement changes
		for (int ii = 0; ii < kempe_chain.size(); ++ii)
		{
			if (ii < kempe_chain.size() - 1)
			{
				if (kempe_chain.at(ii).second == kempe_chain.at(ii + 1).second) // two lectures same room different timeslot (by definition timeslot should be different)
				{
					// swap two lectures
					int lec_temp = current_solution.at(kempe_chain.at(ii).first, kempe_chain.at(ii).second);
					current_solution.at(kempe_chain.at(ii).first, kempe_chain.at(ii).second) = current_solution.at(kempe_chain.at(ii + 1).first, kempe_chain.at(ii + 1).second);
					current_solution.at(kempe_chain.at(ii + 1).first, kempe_chain.at(ii + 1).second) = lec_temp;

					// two lectures swapped, so skip ii+1
					++ii;
				}
				else
				{
					// move lecture in current room to lecture in other room
					if (kempe_chain.at(ii).first == timeslot1)
					{
						int lec_temp = current_solution.at(timeslot1, kempe_chain.at(ii).second);
						current_solution.at(timeslot1, kempe_chain.at(ii).second) = -1;
						current_solution.at(timeslot2, kempe_chain.at(ii).second) = lec_temp;
					}
					else
					{
						int lec_temp = current_solution.at(timeslot2, kempe_chain.at(ii).second);
						current_solution.at(timeslot2, kempe_chain.at(ii).second) = -1;
						current_solution.at(timeslot1, kempe_chain.at(ii).second) = lec_temp;
					}
				}
			}
			else
			{
				// move lecture in current room to lecture in other room
				if (kempe_chain.at(ii).first == timeslot1)
				{
					int lec_temp = current_solution.at(timeslot1, kempe_chain.at(ii).second);
					current_solution.at(timeslot1, kempe_chain.at(ii).second) = -1;
					current_solution.at(timeslot2, kempe_chain.at(ii).second) = lec_temp;
				}
				else
				{
					int lec_temp = current_solution.at(timeslot2, kempe_chain.at(ii).second);
					current_solution.at(timeslot2, kempe_chain.at(ii).second) = -1;
					current_solution.at(timeslot1, kempe_chain.at(ii).second) = lec_temp;
				}
			}
		}


		// 4. Add new values
		{
			// A. TIMETABLE 
			// I. CONSTRAINTS
			// 1. No conflicts
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t == timeslot1 || t == timeslot2)
				{
					int violations = 0;
					for (int r1 = 0; r1 < nb_locations; ++r1)
					{
						for (int r2 = r1 + 1; r2 < nb_locations; ++r2)
						{
							if (solution.at(t, r1) >= 0 && solution.at(t, r2) >= 0 && get_sessionsessionconflict(solution.at(t, r1), solution.at(t, r2)))
								++violations;
						}
					}
					new_constraint_violations_scheduling_conflicts += violations;
				}
			}

			// 2. Correct rooms
			{
				int violations = 0;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (t == timeslot1 || t == timeslot2)
					{
						for (int r = 0; r < nb_locations; ++r)
						{

							if (solution.at(t, r) >= 0 && !get_sessionlocationpossible(solution.at(t, r), r)) // before change to solution
								++violations;

						}
					}
				}
				new_constraint_violations_correct_room += violations;
			}

			// 3.A teacher working time regulations: at most 4 lectures per day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					// only days that change
					if (timeslot1 / nb_timeslots_per_day == day || timeslot2 / nb_timeslots_per_day == day)
					{
						int first_timeslot = day * nb_timeslots_per_day;
						int last_timeslot = (day + 1) * nb_timeslots_per_day;

						int teaches = 0;
						for (int t = first_timeslot; t < last_timeslot; ++t)
						{
							for (int r = 0; r < nb_locations; ++r)
							{
								int lec = solution.at(t, r);
								if (lec >= 0 && get_teachersession(d, lec))
									++teaches;
							}
						}
						if (teaches > 4)
							++new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 3.B teacher working time regulations: no more than 3 lectures consecutively
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					int first_timeslot = day * nb_timeslots_per_day;
					int last_timeslot = first_timeslot + nb_timeslots_per_day - 3;

					int teaches = 0;
					for (int t = first_timeslot; t < last_timeslot; ++t)
					{
						// only if changed timeslots fall in range
						if ((timeslot1 >= t && timeslot1 <= t + 3) || (timeslot2 >= t && timeslot2 <= t + 3))
						{
							bool teaches_ts1 = false, teaches_ts2 = false, teaches_ts3 = false, teaches_ts4 = false;
							for (int r = 0; r < nb_locations; ++r)
							{
								int lec1 = solution.at(t, r);
								int lec2 = solution.at(t + 1, r);
								int lec3 = solution.at(t + 2, r);
								int lec4 = solution.at(t + 3, r);

								if (lec1 >= 0 && get_teachersession(d, lec1))
									++teaches_ts1 = true;
								if (lec2 >= 0 && get_teachersession(d, lec2))
									++teaches_ts2 = true;
								if (lec3 >= 0 && get_teachersession(d, lec3))
									++teaches_ts3 = true;
								if (lec4 >= 0 && get_teachersession(d, lec4))
									++teaches_ts4 = true;
							}
							if (teaches_ts1 && teaches_ts2 && teaches_ts3 && teaches_ts4)
								++new_constraint_violations_teacher_working_time;
						}
					}
				}
			}

			// 3.C teacher working time regulations: not last timeslot of previous day and first timeslot of next day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 1; day < nb_days; ++day)
				{
					int first_timeslot_of_next_day = day * nb_timeslots_per_day;
					int last_timeslot_of_previous_day = day * nb_timeslots_per_day - 1;

					// only if changed timeslots equal one of the two timeslots
					if ((timeslot1 == first_timeslot_of_next_day || timeslot1 == last_timeslot_of_previous_day)
						|| (timeslot2 == first_timeslot_of_next_day || timeslot2 == last_timeslot_of_previous_day))
					{
						bool teaches_t1 = false, teaches_t2 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							int lec1 = solution.at(last_timeslot_of_previous_day, r);
							if (lec1 >= 0 && get_teachersession(d, lec1))
								teaches_t1 = true;

							int lec2 = solution.at(first_timeslot_of_next_day, r);
							if (lec2 >= 0 && get_teachersession(d, lec2))
								teaches_t2 = true;
						}
						if (teaches_t1 && teaches_t2)
							++new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 3.D teacher working time regulations: not first and last timeslot of same day
			for (int d = 0; d < nb_teachers; ++d)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					int first_timeslot_of_day = day * nb_timeslots_per_day;
					int last_timeslot_of_day = (day + 1) * nb_timeslots_per_day - 1;

					// only if changed timeslots equal one of the two timeslots
					if ((timeslot1 == first_timeslot_of_day || timeslot1 == last_timeslot_of_day)
						|| (timeslot2 == first_timeslot_of_day || timeslot2 == last_timeslot_of_day))
					{
						bool teaches_t1 = false, teaches_t2 = false;
						for (int r = 0; r < nb_locations; ++r)
						{
							int lec1 = solution.at(last_timeslot_of_day, r);
							if (lec1 >= 0 && get_teachersession(d, lec1))
								teaches_t1 = true;

							int lec2 = solution.at(first_timeslot_of_day, r);
							if (lec2 >= 0 && get_teachersession(d, lec2))
								teaches_t2 = true;
						}
						if (teaches_t1 && teaches_t2)
							++new_constraint_violations_teacher_working_time;
					}
				}
			}

			// 4. Compactness constraints
			if (nb_timeslots_per_day == 5 || nb_timeslots_per_day == 6)
			{
				for (int s = 0; s < nb_series; ++s)
				{
					for (int day = 0; day < nb_days; ++day)
					{
						for (int t = 0; t < 3; ++t)
						{
							int ts1 = day * nb_timeslots_per_day + t;
							int ts2 = day * nb_timeslots_per_day + t + 1;
							int ts3 = day * nb_timeslots_per_day + t + 2;

							if (timeslot1 == ts1 || timeslot1 == ts2 || timeslot1 == ts3
								|| timeslot2 == ts1 || timeslot2 == ts2 || timeslot2 == ts3)
							{
								bool classts1 = false, classts2 = false, classts3 = false;
								for (int r = 0; r < nb_locations; ++r)
								{
									if (solution.at(ts1, r) >= 0 && get_seriessession(s, solution.at(ts1, r)))
										classts1 = true;
									if (solution.at(ts2, r) >= 0 && get_seriessession(s, solution.at(ts2, r)))
										classts2 = true;
									if (solution.at(ts3, r) >= 0 && get_seriessession(s, solution.at(ts3, r)))
										classts3 = true;
								}

								if (classts1 && !classts2 && classts3)
									++new_constraint_violations_compactness_constraints;
							}
						}
					}
				}
			}
			else
			{
				// N/A 
				current_objective.constraint_violations_compactness_constraints = 0;
			}

			// II. OBJECTIVE (PREFERENCES)
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t == timeslot1 || t == timeslot2)
				{
					for (int r = 0; r < nb_locations; ++r)
					{
						int lec = solution.at(t, r);
						if (lec >= 0)
							new_preference_score += (get_costsessiontimeslot(lec, t) + 1000 * get_costsession_ts_educational(lec, t));
					}
				}
			}
			if (heuristic_master::_constraint_preferences && new_preference_score > heuristic_master::_constraint_preferences_value)
				new_preference_score += heuristic_master::_penalty_value_constraint_violation;
		}



		// 5. calculate flows for relevant timeslots
		// B. FLOWS
		// I. Evacuations
		double objective_evacuations = 0.0;
		{
			current_objective.evacuations_timeslot_lecture_uses_path.fill(0.0);
			current_objective.evacuations_timeslot_flow_per_arc.fill(0.0);
			current_objective.evacuations_timeslot_time_per_arc.fill(0.0);
			current_objective.evacuations_timeslot_lecture_travel_time.fill(0.0);
			new_max_evac_time_changed_ts.at(0) = 0;
			new_max_evac_time_changed_ts.at(1) = 0;

			int index_ts = 0;
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t == timeslot1 || t == timeslot2)
				{
					// 1. who uses which paths
					for (int r = 0; r < nb_locations; ++r)
					{
						int lec = solution.at(t, r);
						if (lec >= 0)
						{
							for (int p = 0; p < nb_paths; ++p)
								current_objective.evacuations_timeslot_lecture_uses_path.at(t, lec, p) = get_roomroompath(r, nb_locations, p);
						}
					}

					// 2. flow per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						for (int l = 0; l < nb_sessions; ++l)
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								double nbp = current_objective.evacuations_timeslot_lecture_uses_path.at(t, l, p) * session_nb_people.at(l);
								if (get_patharc(p, ij))
									current_objective.evacuations_timeslot_flow_per_arc.at(t, ij) += nbp;
							}
						}
					}

					// 3. time per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						current_objective.evacuations_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * current_objective.evacuations_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
						if (arc_stairs.at(ij))
							current_objective.evacuations_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
					}

					// 4. time per lecture in total
					for (int l = 0; l < nb_sessions; ++l)
					{
						double time_longest_path = 0.0;
						for (int p = 0; p < nb_paths; ++p)
						{
							if (current_objective.evacuations_timeslot_lecture_uses_path.at(t, l, p) > 0.01)
							{
								double time_current_path = 0.0;
								for (int ij = 0; ij < nb_arcs; ++ij)
								{
									if (get_patharc(p, ij))
										time_current_path += current_objective.evacuations_timeslot_time_per_arc.at(t, ij);
								}
								if (time_current_path > time_longest_path)
									time_longest_path = time_current_path;
							}
						}
						current_objective.evacuations_timeslot_lecture_travel_time.at(t, l) = time_longest_path;
					}

					// 5. maximum evacuation time per timeslot
					for (int l = 0; l < nb_sessions; ++l)
					{
						if (current_objective.evacuations_timeslot_lecture_travel_time.at(t, l) > new_max_evac_time_changed_ts.at(index_ts))
							new_max_evac_time_changed_ts.at(index_ts) = current_objective.evacuations_timeslot_lecture_travel_time.at(t, l);
					}

					++index_ts; // go to next timeslot
				}
			}

			// 6. objective evacuations
			if (heuristic_master::_objective_type == heuristic_master::objective_type::maximum_over_timeslots)
			{
				index_ts = 0;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (t == timeslot1 || t == timeslot2)
					{
						if (new_max_evac_time_changed_ts.at(index_ts) > objective_evacuations)
							objective_evacuations = new_max_evac_time_changed_ts.at(index_ts);
						++index_ts;
					}
					else
					{
						if (current_objective.evacuations_timeslot_max_travel_time.at(t) > objective_evacuations)
							objective_evacuations = current_objective.evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				index_ts = 0;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (t == timeslot1 || t == timeslot2)
					{
						objective_evacuations += new_max_evac_time_changed_ts.at(index_ts);
						++index_ts;
					}
					else
					{
						objective_evacuations += current_objective.evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
		}



		// 6. Undo changes
		for (int ii = 0; ii < kempe_chain.size(); ++ii)
		{
			if (ii < kempe_chain.size() - 1)
			{
				if (kempe_chain.at(ii).second == kempe_chain.at(ii + 1).second) // two lectures same room different timeslot (by definition timeslot should be different)
				{
					// swap two lectures
					int lec_temp = current_solution.at(kempe_chain.at(ii).first, kempe_chain.at(ii).second);
					current_solution.at(kempe_chain.at(ii).first, kempe_chain.at(ii).second) = current_solution.at(kempe_chain.at(ii + 1).first, kempe_chain.at(ii + 1).second);
					current_solution.at(kempe_chain.at(ii + 1).first, kempe_chain.at(ii + 1).second) = lec_temp;

					// two lectures swapped, so skip ii+1
					++ii;
				}
				else
				{
					// move lecture in current room to lecture in other room
					if (kempe_chain.at(ii).first == timeslot1)
					{
						int lec_temp = current_solution.at(timeslot2, kempe_chain.at(ii).second); // different timeslot because we undo swap
						current_solution.at(timeslot2, kempe_chain.at(ii).second) = -1;
						current_solution.at(timeslot1, kempe_chain.at(ii).second) = lec_temp;
					}
					else
					{
						int lec_temp = current_solution.at(timeslot1, kempe_chain.at(ii).second);  // different timeslot because we undo swap
						current_solution.at(timeslot1, kempe_chain.at(ii).second) = -1;
						current_solution.at(timeslot2, kempe_chain.at(ii).second) = lec_temp;
					}
				}
			}
			else
			{
				// move lecture in current room to lecture in other room
				if (kempe_chain.at(ii).first == timeslot1)
				{
					int lec_temp = current_solution.at(timeslot2, kempe_chain.at(ii).second);  // different timeslot because we undo swap
					current_solution.at(timeslot2, kempe_chain.at(ii).second) = -1;
					current_solution.at(timeslot1, kempe_chain.at(ii).second) = lec_temp;
				}
				else
				{
					int lec_temp = current_solution.at(timeslot1, kempe_chain.at(ii).second);  // different timeslot because we undo swap
					current_solution.at(timeslot1, kempe_chain.at(ii).second) = -1;
					current_solution.at(timeslot2, kempe_chain.at(ii).second) = lec_temp;
				}
			}
		}



		// 7. recalculate objective value
		return (heuristic_master::_penalty_value_constraint_violation * (new_constraint_violations_scheduling_conflicts
			+ new_constraint_violations_correct_room
			+ new_constraint_violations_teacher_working_time
			+ new_constraint_violations_compactness_constraints)
			+ heuristic_master::_lambda * new_preference_score + (1 - heuristic_master::_lambda) * objective_evacuations);
	}
}
