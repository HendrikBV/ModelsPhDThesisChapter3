#include "heuristic.h"
#include "timetable_data.h"
#include "building_data.h"
#include "initial_solution.h"
#include "logger.h"
#include "MIP_two_stage.h"

#include <iostream>
#include <random>
#include <utility>



namespace
{
	bool sort_on_room(const std::pair<int, int>& a, const std::pair<int, int>& b)
	{
		if (a.second < b.second)
			return true;

		return false;
	}
}



namespace alg
{

	void heuristic::run()
	{
		initialize();
		LAHC();

		write_output();
		//clear();
	}


	void heuristic::initialize()
	{
		current_solution.set(nb_timeslots, nb_locations, -1);

		evacuations_timeslot_lecture_uses_path.set(nb_timeslots, nb_sessions, nb_paths, 0.0);
		evacuations_timeslot_flow_per_arc.set(nb_timeslots, nb_arcs, 0.0);
		evacuations_timeslot_time_per_arc.set(nb_timeslots, nb_arcs, 0.0);
		evacuations_timeslot_lecture_travel_time.set(nb_timeslots, nb_sessions, 0.0);
		evacuations_timeslot_max_travel_time.reserve(nb_timeslots);
		for (int i = 0; i < nb_timeslots; ++i) evacuations_timeslot_max_travel_time.push_back(0.0);

		travels_timeslot_series_uses_path.set(nb_timeslots, nb_series, nb_paths, 0.0);
		travels_timeslot_flow_per_arc.set(nb_timeslots, nb_arcs, 0.0);
		travels_timeslot_time_per_arc.set(nb_timeslots, nb_arcs, 0.0);
		travels_timeslot_series_travel_time.set(nb_timeslots, nb_series, 0.0);
		travels_timeslot_max_travel_time.reserve(nb_timeslots);
		for (int i = 0; i < nb_timeslots; ++i) travels_timeslot_max_travel_time.push_back(0.0);

		if (nb_timeslots_per_day == 5)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t % nb_timeslots_per_day != 1 && t % nb_timeslots_per_day != 4)
					_relevant_timeslots.push_back(t);
			}
		}
		else if (nb_timeslots_per_day == 6)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t % nb_timeslots_per_day != 1 && t % nb_timeslots_per_day != 5)
					_relevant_timeslots.push_back(t);
			}
		}
		else
		{
			for (int t = 0; t < nb_timeslots; ++t)
				_relevant_timeslots.push_back(t);
		}
	}


	void heuristic::clear()
	{
		current_solution.clear();
		best_solution.clear();

		evacuations_timeslot_lecture_uses_path.clear();
		evacuations_timeslot_flow_per_arc.clear();
		evacuations_timeslot_time_per_arc.clear();
		evacuations_timeslot_lecture_travel_time.clear();
		evacuations_timeslot_max_travel_time.clear();

		travels_timeslot_series_uses_path.clear();
		travels_timeslot_flow_per_arc.clear();
		travels_timeslot_time_per_arc.clear();
		travels_timeslot_series_travel_time.clear();
		travels_timeslot_max_travel_time.clear();
	}


	void heuristic::write_output()
	{
		std::string text;

		// 1. instance name
		text = "Algorithm: "; text.append(algorithm_name);
		text += "\nProblem: " + instance_name_timetable + " + " + instance_name_building;

		// 2. settings
		text += "\nSettings:";
		text += "\n\tLambda: " + std::to_string(_lambda);
		text += "\n\tAlpha: " + std::to_string(_alpha);
		text += "\n\tObjective type: ";
		if (_objective_type == objective_type::maximum_over_timeslots)
			text += "maximum over timeslots";
		else
			text += "sum over timeslots";
		text += "\n\tTime limit (seconds): " + std::to_string(_time_limit);
		text += "\n\tLAHC list length: " + std::to_string(_LAHC_list_length);

		// 4. results
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



	////////////////////////////////////////////////////////////////////////////////////////////

	void heuristic::LAHC()
	{
		// 0. start timer
		std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

		// 1. Initialize random number generator and move probabilities
		std::random_device randdev;
		std::seed_seq seedseq{ randdev(),randdev(),randdev(),randdev(),randdev(),randdev(),randdev() };
		std::mt19937_64 generator(seedseq);
		double probability_move[] = { 0.6, 0.38, 0.02 };


		// 2. constructive method: MIP
		{
			MIP_two_stage MIP;
			MIP.run_only_first_stage();
			if (!MIP.get_solution_firststage_exists())
				throw std::runtime_error("Error in function heuristic::LAHC(). \nCouldn't construct a feasible solution.");

			for (int l = 0; l < nb_sessions; ++l)
			{
				int timeslot = MIP.get_solution_lecture_timeslot(l);
				int room = MIP.get_solution_lecture_room(l);

				current_solution.at(timeslot, room) = l;
			}
		}
		current_solution.print();
		best_solution = current_solution;
		current_objective = evaluate(current_solution);
		best_objective = current_objective;
		std::cout << "\n\nObjective value initial solution: " << current_objective;


		// 3. initialize LAHC_list
		std::vector<double> LAHC_list;
		LAHC_list.reserve(_LAHC_list_length);
		for (int i = 0; i < _LAHC_list_length; ++i)
			LAHC_list.push_back(current_objective);


		// 4. main LAHC loop
		size_t iteration = 0;
		while (true)
		{
			// new iteration
			++iteration;
			if (iteration % 1000 == 0)
			{
				std::cout << "\n\nIteration: " << iteration;
				std::cout << "\nObjective value current solution: " << current_objective;
				std::cout << "\nObjective value best solution: " << best_objective;
			}

			// time check
			std::chrono::duration<double, std::ratio<1, 1>> elapsed_time = std::chrono::system_clock::now() - start_time;
			if (elapsed_time.count() > _time_limit)
			{
				std::cout << "\n\nLAHC heuristic has reached time limit of " << _time_limit << " seconds.";
				std::cout << "\nIterations: " << iteration;
				std::cout << "\nObjective value best solution: " << best_objective;
				break;
			}

			// choose a move
			std::uniform_real_distribution<double> prob_dist(0, 1);
			double prob = prob_dist(generator);




			// A. Lecture swap
			if (prob < probability_move[0])
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

				double candidate_objective = evaluate_incremental(current_solution, timeslot1, timeslot2, room1, room2);

				// accept if better than current solution or better than value in list
				if (candidate_objective < current_objective || candidate_objective < LAHC_list[iteration % _LAHC_list_length])
				{
					// save the objective value
					current_objective = candidate_objective;

					// update the values for individual objectives and constraints
					constraint_violations_scheduling_conflicts = new_constraint_violations_scheduling_conflicts;
					constraint_violations_correct_room = new_constraint_violations_correct_room;
					constraint_violations_teacher_working_time = new_constraint_violations_teacher_working_time;
					constraint_violations_compactness_constraints = new_constraint_violations_compactness_constraints;
					preference_score = new_preference_score;

					// update the evacuation and travel times
					if (timeslot2 < timeslot1)
						std::swap(timeslot1, timeslot2);

					{
						evacuations_timeslot_max_travel_time.at(timeslot1) = new_max_evac_time_changed_ts.at(0);
						evacuations_timeslot_max_travel_time.at(timeslot2) = new_max_evac_time_changed_ts.at(1);

						if (timeslot1 > 0 && timeslot2 < nb_timeslots - 1)
						{
							travels_timeslot_max_travel_time.at(timeslot1 - 1) = new_max_travel_time_changes_ts.at(0);
							travels_timeslot_max_travel_time.at(timeslot1) = new_max_travel_time_changes_ts.at(1);
							travels_timeslot_max_travel_time.at(timeslot2) = new_max_travel_time_changes_ts.at(2);
							travels_timeslot_max_travel_time.at(timeslot2 + 1) = new_max_travel_time_changes_ts.at(3);
						}
						else if (timeslot1 > 0)
						{
							travels_timeslot_max_travel_time.at(timeslot1 - 1) = new_max_travel_time_changes_ts.at(0);
							travels_timeslot_max_travel_time.at(timeslot1) = new_max_travel_time_changes_ts.at(1);
							travels_timeslot_max_travel_time.at(timeslot2) = new_max_travel_time_changes_ts.at(2);
						}
						else if (timeslot2 < nb_timeslots - 1)
						{
							travels_timeslot_max_travel_time.at(timeslot1) = new_max_travel_time_changes_ts.at(0);
							travels_timeslot_max_travel_time.at(timeslot2) = new_max_travel_time_changes_ts.at(1);
							travels_timeslot_max_travel_time.at(timeslot2 + 1) = new_max_travel_time_changes_ts.at(2);
						}
						else
						{
							travels_timeslot_max_travel_time.at(timeslot1) = new_max_travel_time_changes_ts.at(0);
							travels_timeslot_max_travel_time.at(timeslot2) = new_max_travel_time_changes_ts.at(1);
						}
					}

					// implement the change
					int lec_temp = current_solution.at(timeslot1, room1);
					current_solution.at(timeslot1, room1) = current_solution.at(timeslot2, room2);
					current_solution.at(timeslot2, room2) = lec_temp;

					if (current_objective < best_objective)
					{
						best_objective = current_objective;
						best_solution = current_solution;
						std::cout << "\n\nNew best solution found (Lecture Swap)!\nObjective value " << current_objective;

						//best_solution.print();
					}
				}
			}





			// B. Room Swap
			else if (prob < probability_move[0] + probability_move[1])
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

				// accept if better than current solution or better than value in list
				if (candidate_objective < current_objective || candidate_objective < LAHC_list[iteration % _LAHC_list_length])
				{
					// update the objective value
					current_objective = candidate_objective;

					// update the values for individual objectives and constraints
					constraint_violations_scheduling_conflicts = new_constraint_violations_scheduling_conflicts;
					constraint_violations_correct_room = new_constraint_violations_correct_room;
					constraint_violations_teacher_working_time = new_constraint_violations_teacher_working_time;
					constraint_violations_compactness_constraints = new_constraint_violations_compactness_constraints;
					preference_score = new_preference_score;

					// save the new evacuation/travel times
					{
						evacuations_timeslot_max_travel_time.at(timeslot) = new_max_evac_time_changed_ts_roomswap;

						if (timeslot > 0)
						{
							travels_timeslot_max_travel_time.at(timeslot - 1) = new_max_travel_time_changes_ts_roomswap.at(0);
							travels_timeslot_max_travel_time.at(timeslot) = new_max_travel_time_changes_ts_roomswap.at(1);
						}
						else
						{
							travels_timeslot_max_travel_time.at(timeslot) = new_max_travel_time_changes_ts_roomswap.at(0);
						}
					}

					// implement the change
					int lec_temp = current_solution.at(timeslot, room1);
					current_solution.at(timeslot, room1) = current_solution.at(timeslot, room2);
					current_solution.at(timeslot, room2) = lec_temp;

					if (current_objective < best_objective)
					{
						best_objective = current_objective;
						best_solution = current_solution;
						std::cout << "\n\nNew best solution found (Room Swap)!\nObjective value " << current_objective;

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
						double candidate_objective = evaluate_incremental(current_solution, current_objective, kempe_chains.at(i), timeslot1, timeslot2);

						// save if improvement
						if (candidate_objective < best_obj_val_KC) // best chain of all chains
						{
							best_obj_val_KC = candidate_objective;

							if (candidate_objective < current_objective || candidate_objective < LAHC_list[iteration % _LAHC_list_length]) // improvement to current solution
							{
								best_index_KC = i;
							}
						}
					}


					// (4) implement best chain if improvement
					if (best_index_KC != -1)
					{
						// save the objective value
						current_objective = best_obj_val_KC;

						// update the values for individual objectives and constraints
						constraint_violations_scheduling_conflicts = new_constraint_violations_scheduling_conflicts;
						constraint_violations_correct_room = new_constraint_violations_correct_room;
						constraint_violations_teacher_working_time = new_constraint_violations_teacher_working_time;
						constraint_violations_compactness_constraints = new_constraint_violations_compactness_constraints;
						preference_score = new_preference_score;

						// update the evacuation/travel times
						{
							// recalculate so that correct values are in the new vectors
							double candidate_objective = evaluate_incremental(current_solution, current_objective, kempe_chains.at(best_index_KC), timeslot1, timeslot2);

							if (timeslot2 < timeslot1)
								std::swap(timeslot1, timeslot2);

							{
								evacuations_timeslot_max_travel_time.at(timeslot1) = new_max_evac_time_changed_ts.at(0);
								evacuations_timeslot_max_travel_time.at(timeslot2) = new_max_evac_time_changed_ts.at(1);

								if (timeslot1 > 0 && timeslot2 < nb_timeslots - 1)
								{
									travels_timeslot_max_travel_time.at(timeslot1 - 1) = new_max_travel_time_changes_ts.at(0);
									travels_timeslot_max_travel_time.at(timeslot1) = new_max_travel_time_changes_ts.at(1);
									travels_timeslot_max_travel_time.at(timeslot2) = new_max_travel_time_changes_ts.at(2);
									travels_timeslot_max_travel_time.at(timeslot2 + 1) = new_max_travel_time_changes_ts.at(3);
								}
								else if (timeslot1 > 0)
								{
									travels_timeslot_max_travel_time.at(timeslot1 - 1) = new_max_travel_time_changes_ts.at(0);
									travels_timeslot_max_travel_time.at(timeslot1) = new_max_travel_time_changes_ts.at(1);
									travels_timeslot_max_travel_time.at(timeslot2) = new_max_travel_time_changes_ts.at(2);
								}
								else if (timeslot2 < nb_timeslots - 1)
								{
									travels_timeslot_max_travel_time.at(timeslot1) = new_max_travel_time_changes_ts.at(0);
									travels_timeslot_max_travel_time.at(timeslot2) = new_max_travel_time_changes_ts.at(1);
									travels_timeslot_max_travel_time.at(timeslot2 + 1) = new_max_travel_time_changes_ts.at(2);
								}
								else
								{
									travels_timeslot_max_travel_time.at(timeslot1) = new_max_travel_time_changes_ts.at(0);
									travels_timeslot_max_travel_time.at(timeslot2) = new_max_travel_time_changes_ts.at(1);
								}
							}
						}



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

						if (current_objective < best_objective)
						{
							best_objective = current_objective;
							best_solution = current_solution;
							std::cout << "\n\nNew best solution found (Kempe Chain)!\nObjective value " << current_objective;

							//best_solution.print();
						}
					}
				}
			}




			// put the value of the (new) current solution in the LAHC list
			LAHC_list.at(iteration % _LAHC_list_length) = current_objective;
		}
	}



	////////////////////////////////////////////////////////////////////////////////////////////

	double heuristic::evaluate(const matrix2D<int>& solution)
	{
		// A. TIMETABLE 
		// I. CONSTRAINTS
		// 1. No conflicts
		constraint_violations_scheduling_conflicts = 0;
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
			constraint_violations_scheduling_conflicts += violations;
		}

		// 2. Correct rooms
		constraint_violations_correct_room = 0;
		for (int t = 0; t < nb_timeslots; ++t)
		{
			int violations = 0;
			for (int r = 0; r < nb_locations; ++r)
			{
				int lec = solution.at(t, r);
				if (lec >= 0 && !get_sessionlocationpossible(lec, r))
					++violations;

			}
			constraint_violations_correct_room += violations;
		}

		// 3. Teacher working time regulations
		constraint_violations_teacher_working_time = 0;

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
					++constraint_violations_teacher_working_time;
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
						++constraint_violations_teacher_working_time;
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
					++constraint_violations_teacher_working_time;
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
					++constraint_violations_teacher_working_time;
			}
		}

		// 4. Compactness constraints
		if (nb_timeslots_per_day == 5 || nb_timeslots_per_day == 6)
		{
			constraint_violations_compactness_constraints = 0;
			for (int s = 0; s < nb_series; ++s)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					for (int t = 2; t < 3; ++t) // (1-3, 2-4,) 3-5
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
							++constraint_violations_compactness_constraints;
					}
				}
			}
		}
		else
		{
			// N/A 
			constraint_violations_compactness_constraints = 0;
		}



		// II. OBJECTIVE (PREFERENCES)
		preference_score = 0;
		for (int t = 0; t < nb_timeslots; ++t)
		{
			int pref = 0;
			for (int r = 0; r < nb_locations; ++r)
			{
				int lec = solution.at(t, r);
				if (lec >= 0)
					pref += (get_costsessiontimeslot(lec, t) + 1000 * get_costsession_ts_educational(lec, t));
			}
			preference_score += pref;
		}
		if (_constraint_preferences && preference_score > _constraint_preferences_value)
			preference_score += _penalty_value_constraint_violation;



		// B. FLOWS
		// I. Evacuations
		double objective_evacuations = 0.0;
		if (_alpha > 0.0001)
		{
			evacuations_timeslot_lecture_uses_path.fill(0.0);
			evacuations_timeslot_flow_per_arc.fill(0.0);
			evacuations_timeslot_time_per_arc.fill(0.0);
			evacuations_timeslot_lecture_travel_time.fill(0.0);
			for (int t = 0; t < nb_timeslots; ++t)
				evacuations_timeslot_max_travel_time.at(t) = 0.0;

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
							evacuations_timeslot_lecture_uses_path.at(t, lec, p) = get_roomroompath(r, nb_locations, p);
					}
				}

				// 2. flow per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					for (int l = 0; l < nb_sessions; ++l)
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							double nbp = evacuations_timeslot_lecture_uses_path.at(t, l, p) * session_nb_people.at(l);
							if (get_patharc(p, ij))
								evacuations_timeslot_flow_per_arc.at(t, ij) += nbp;
						}
					}
				}
				//std::cout << "\n\n\nevacuations_timeslot_flow_per_arc\n\n";
				//evacuations_timeslot_flow_per_arc.print();

				// 3. time per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					evacuations_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * evacuations_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
					if (arc_stairs.at(ij))
						evacuations_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
				}
				//std::cout << "\n\n\nevacuations_timeslot_time_per_arc\n\n";
				//evacuations_timeslot_time_per_arc.print();

				// 4. time per lecture in total
				for (int l = 0; l < nb_sessions; ++l)
				{
					double time_longest_path = 0.0;
					for (int p = 0; p < nb_paths; ++p)
					{
						if (evacuations_timeslot_lecture_uses_path.at(t, l, p) > 0.01)
						{
							double time_current_path = 0.0;
							for (int ij = 0; ij < nb_arcs; ++ij)
							{
								if (get_patharc(p, ij))
									time_current_path += evacuations_timeslot_time_per_arc.at(t, ij);
							}
							if (time_current_path > time_longest_path)
								time_longest_path = time_current_path;
						}
					}
					evacuations_timeslot_lecture_travel_time.at(t, l) = time_longest_path;
				}
				//std::cout << "\n\n\nevacuations_timeslot_lecture_travel_time\n\n";
				//evacuations_timeslot_lecture_travel_time.print();

				// 5. maximum evacuation time per timeslot
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (evacuations_timeslot_lecture_travel_time.at(t, l) > evacuations_timeslot_max_travel_time.at(t))
						evacuations_timeslot_max_travel_time.at(t) = evacuations_timeslot_lecture_travel_time.at(t, l);
				}
			}

			// 6. objective evacuations
			if (_objective_type == objective_type::maximum_over_timeslots)
			{
				for (int t = 0; t < nb_timeslots; ++t)
					if (evacuations_timeslot_max_travel_time.at(t) > objective_evacuations)
						objective_evacuations = evacuations_timeslot_max_travel_time.at(t);
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				for (int t = 0; t < nb_timeslots; ++t)
					objective_evacuations += evacuations_timeslot_max_travel_time.at(t);
			}
		}




		// II. Flows between events in consecutive timeslots
		double objective_travels = 0.0;
		if (_alpha < 0.9999)
		{
			travels_timeslot_series_uses_path.fill(0.0);
			travels_timeslot_flow_per_arc.fill(0.0);
			travels_timeslot_time_per_arc.fill(0.0);
			travels_timeslot_series_travel_time.fill(0.0);
			for (int t = 0; t < nb_timeslots; ++t)
				travels_timeslot_max_travel_time.at(t) = 0.0;

			for (int t = 0; t < nb_timeslots - 1; ++t) // not last timeslot
			{
				// 1. who uses which paths
				for (int s = 0; s < nb_series; ++s)
				{
					// a) lecture t and t + 1
					for (int l1 = 0; l1 < nb_sessions; ++l1)
					{
						if (get_seriessession(s, l1))
						{
							for (int r1 = 0; r1 < nb_locations; ++r1)
							{
								if (solution.at(t, r1) == l1) // current timeslot
								{
									for (int l2 = 0; l2 < nb_sessions; ++l2)
									{
										if (get_seriessession(s, l2))
										{
											for (int r2 = 0; r2 < nb_locations; ++r2)
											{
												if (solution.at(t + 1, r2) == l2) // next timeslot
												{
													for (int p = 0; p < nb_paths; ++p)
														travels_timeslot_series_uses_path.at(t, s, p) = get_roomroompath(r1, r2, p);
												}
											}
										}
									}
								}
							}
						}
					}

					// b) lecture t but not t + 1
					for (int l1 = 0; l1 < nb_sessions; ++l1)
					{
						if (get_seriessession(s, l1))
						{
							for (int r1 = 0; r1 < nb_locations; ++r1)
							{
								if (solution.at(t, r1) == l1) // current timeslot
								{
									bool lecture_followed_t2 = false;
									for (int l2 = 0; l2 < nb_sessions; ++l2) // next timeslot
									{
										for (int r2 = 0; r2 < nb_locations; ++r2)
										{
											int lec = solution.at(t + 1, r2);
											if (lec >= 0 && get_seriessession(s, lec))
											{
												lecture_followed_t2 = true;
												break;
											}
										}
									}
									if (!lecture_followed_t2)
									{
										for (int p = 0; p < nb_paths; ++p)
											travels_timeslot_series_uses_path.at(t, s, p) = -get_roomroompath(r1, nb_locations, p); // negative to remember to exclude in T_max
									}
								}
							}
						}
					}

					// c) lecture t + 1 but not t
					for (int l2 = 0; l2 < nb_sessions; ++l2)
					{
						if (get_seriessession(s, l2))
						{
							for (int r2 = 0; r2 < nb_locations; ++r2)
							{
								if (solution.at(t + 1, r2) == l2) // next timeslot
								{
									bool lecture_followed_t1 = false;
									for (int l1 = 0; l1 < nb_sessions; ++l1) // current timeslot
									{
										for (int r1 = 0; r1 < nb_locations; ++r1)
										{
											int lec = solution.at(t, r1);
											if (lec >= 0 && get_seriessession(s, lec))
											{
												lecture_followed_t1 = true;
												break;
											}
										}
									}
									if (!lecture_followed_t1)
									{
										for (int p = 0; p < nb_paths; ++p)
											travels_timeslot_series_uses_path.at(t, s, p) = -get_roomroompath(r2, nb_locations, p); // negative to remember to exclude in T_max
									}
								}
							}
						}
					}
				}

				// 2. flow per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					for (int s = 0; s < nb_series; ++s)
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							double nbp = std::abs(travels_timeslot_series_uses_path.at(t, s, p)) * series_nb_people.at(s); // absolute value because U_tsp can be negative 
							if (get_patharc(p, ij))
								travels_timeslot_flow_per_arc.at(t, ij) += nbp;
						}
					}
				}
				//std::cout << "\n\n\ntravels_timeslot_flow_per_arc\n\n";
				//travels_timeslot_flow_per_arc.print();

				// 3. time per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					travels_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * travels_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
					if (arc_stairs.at(ij))
						travels_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
				}
				//std::cout << "\n\n\ntravels_timeslot_time_per_arc\n\n";
				//travels_timeslot_time_per_arc.print();

				// 4. time per series in total
				for (int s = 0; s < nb_series; ++s)
				{
					double time_longest_path = 0.0;
					for (int p = 0; p < nb_paths; ++p)
					{
						if (travels_timeslot_series_uses_path.at(t, s, p) > 0.01) // excludes unused paths and paths towards exits (which have negative values)
						{
							double time_current_path = 0.0;
							for (int ij = 0; ij < nb_arcs; ++ij)
							{
								if (get_patharc(p, ij))
									time_current_path += travels_timeslot_time_per_arc.at(t, ij);
							}
							if (time_current_path > time_longest_path)
								time_longest_path = time_current_path;
						}
					}
					travels_timeslot_series_travel_time.at(t, s) = time_longest_path;
				}
				//std::cout << "\n\n\ntravels_timeslot_series_travel_time\n\n";
				//travels_timeslot_series_travel_time.print();

				// 5. maximum travel time per timeslot
				for (int s = 0; s < nb_series; ++s)
				{
					if (travels_timeslot_series_travel_time.at(t, s) > travels_timeslot_max_travel_time.at(t))
						travels_timeslot_max_travel_time.at(t) = travels_timeslot_series_travel_time.at(t, s);
				}
			}

			// 6. objective travels
			if (_objective_type == objective_type::maximum_over_timeslots)
			{
				for (auto&& t : _relevant_timeslots)
					if (travels_timeslot_max_travel_time.at(t) > objective_travels)
						objective_travels = travels_timeslot_max_travel_time.at(t);
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				for (auto&& t : _relevant_timeslots)
					objective_travels += travels_timeslot_max_travel_time.at(t);
			}
		}


		double objective_flows = (_alpha * objective_evacuations + (1 - _alpha) * objective_travels);

		return (_penalty_value_constraint_violation * (constraint_violations_scheduling_conflicts
			+ constraint_violations_correct_room
			+ constraint_violations_teacher_working_time
			+ constraint_violations_compactness_constraints)
			+ _lambda * preference_score + (1 - _lambda) * objective_flows);
	}



	////////////////////////////////////////////////////////////////////////////////////////////

	double heuristic::evaluate_incremental(matrix2D<int>& solution, int timeslot, int room1, int room2)
	{
		// 1. start from current solution
		new_constraint_violations_scheduling_conflicts = constraint_violations_scheduling_conflicts;
		new_constraint_violations_correct_room = constraint_violations_correct_room;
		new_constraint_violations_teacher_working_time = constraint_violations_correct_room;
		new_constraint_violations_compactness_constraints = constraint_violations_compactness_constraints;
		new_preference_score = preference_score;

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

				if (solution.at(timeslot, room1) >= 0 && !get_sessionlocationpossible(solution.at(timeslot, room1), room1)) // before change to solution
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
		if (_alpha > 0.0001)
		{
			evacuations_timeslot_lecture_uses_path.fill(0.0);
			evacuations_timeslot_flow_per_arc.fill(0.0);
			evacuations_timeslot_time_per_arc.fill(0.0);
			evacuations_timeslot_lecture_travel_time.fill(0.0);
			new_max_evac_time_changed_ts_roomswap = 0.0;

			{
				// 1. who uses which paths
				for (int r = 0; r < nb_locations; ++r)
				{
					int lec = solution.at(timeslot, r);
					if (lec >= 0)
					{
						for (int p = 0; p < nb_paths; ++p)
							evacuations_timeslot_lecture_uses_path.at(timeslot, lec, p) = get_roomroompath(r, nb_locations, p);
					}
				}

				// 2. flow per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					for (int l = 0; l < nb_sessions; ++l)
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							double nbp = evacuations_timeslot_lecture_uses_path.at(timeslot, l, p) * session_nb_people.at(l);
							if (get_patharc(p, ij))
								evacuations_timeslot_flow_per_arc.at(timeslot, ij) += nbp;
						}
					}
				}

				// 3. time per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					evacuations_timeslot_time_per_arc.at(timeslot, ij) = arc_length.at(ij) / walking_alpha * evacuations_timeslot_flow_per_arc.at(timeslot, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
					if (arc_stairs.at(ij))
						evacuations_timeslot_time_per_arc.at(timeslot, ij) *= speed_correction_stairs;
				}

				// 4. time per lecture in total
				for (int l = 0; l < nb_sessions; ++l)
				{
					double time_longest_path = 0.0;
					for (int p = 0; p < nb_paths; ++p)
					{
						if (evacuations_timeslot_lecture_uses_path.at(timeslot, l, p) > 0.01)
						{
							double time_current_path = 0.0;
							for (int ij = 0; ij < nb_arcs; ++ij)
							{
								if (get_patharc(p, ij))
									time_current_path += evacuations_timeslot_time_per_arc.at(timeslot, ij);
							}
							if (time_current_path > time_longest_path)
								time_longest_path = time_current_path;
						}
					}
					evacuations_timeslot_lecture_travel_time.at(timeslot, l) = time_longest_path;
				}

				// 5. maximum evacuation time per timeslot
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (evacuations_timeslot_lecture_travel_time.at(timeslot, l) > new_max_evac_time_changed_ts_roomswap)
						new_max_evac_time_changed_ts_roomswap = evacuations_timeslot_lecture_travel_time.at(timeslot, l);
				}
			}

			// 6. objective evacuations
			if (_objective_type == objective_type::maximum_over_timeslots)
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
						if (evacuations_timeslot_max_travel_time.at(t) > objective_evacuations)
							objective_evacuations = evacuations_timeslot_max_travel_time.at(t);
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
						objective_evacuations += evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
		}



		// II. Flows between events in consecutive timeslots
		double objective_travels = 0.0;
		if (_alpha < 0.9999)
		{
			travels_timeslot_series_uses_path.fill(0.0);
			travels_timeslot_flow_per_arc.fill(0.0);
			travels_timeslot_time_per_arc.fill(0.0);
			travels_timeslot_series_travel_time.fill(0.0);
			new_max_travel_time_changes_ts_roomswap.at(0) = 0;
			new_max_travel_time_changes_ts_roomswap.at(1) = 0;

			int first_timeslot = std::max(timeslot - 1, 0); // not -1 or smaller
			int second_timeslot = std::min(nb_timeslots - 1, timeslot + 1); // not nb_timeslots or larger
			int index_ts = 0;

			for (int t = first_timeslot; t < second_timeslot; ++t)
			{
				// 1. who uses which paths
				for (int s = 0; s < nb_series; ++s)
				{
					// a) lecture t and t + 1
					for (int l1 = 0; l1 < nb_sessions; ++l1)
					{
						if (get_seriessession(s, l1))
						{
							for (int r1 = 0; r1 < nb_locations; ++r1)
							{
								if (solution.at(t, r1) == l1) // current timeslot
								{
									for (int l2 = 0; l2 < nb_sessions; ++l2)
									{
										if (get_seriessession(s, l2))
										{
											for (int r2 = 0; r2 < nb_locations; ++r2)
											{
												if (solution.at(t + 1, r2) == l2) // next timeslot
												{
													for (int p = 0; p < nb_paths; ++p)
														travels_timeslot_series_uses_path.at(t, s, p) = get_roomroompath(r1, r2, p);
												}
											}
										}
									}
								}
							}
						}
					}

					// b) lecture t but not t + 1
					for (int l1 = 0; l1 < nb_sessions; ++l1)
					{
						if (get_seriessession(s, l1))
						{
							for (int r1 = 0; r1 < nb_locations; ++r1)
							{
								if (solution.at(t, r1) == l1) // current timeslot
								{
									bool lecture_followed_t2 = false;
									for (int l2 = 0; l2 < nb_sessions; ++l2) // next timeslot
									{
										for (int r2 = 0; r2 < nb_locations; ++r2)
										{
											int lec = solution.at(t + 1, r2);
											if (lec >= 0 && get_seriessession(s, lec))
											{
												lecture_followed_t2 = true;
												break;
											}
										}
									}
									if (!lecture_followed_t2)
									{
										for (int p = 0; p < nb_paths; ++p)
											travels_timeslot_series_uses_path.at(t, s, p) = -get_roomroompath(r1, nb_locations, p); // negative to remember to exclude in T_max
									}
								}
							}
						}
					}

					// c) lecture t + 1 but not t
					for (int l2 = 0; l2 < nb_sessions; ++l2)
					{
						if (get_seriessession(s, l2))
						{
							for (int r2 = 0; r2 < nb_locations; ++r2)
							{
								if (solution.at(t + 1, r2) == l2) // next timeslot
								{
									bool lecture_followed_t1 = false;
									for (int l1 = 0; l1 < nb_sessions; ++l1) // current timeslot
									{
										for (int r1 = 0; r1 < nb_locations; ++r1)
										{
											int lec = solution.at(t, r1);
											if (lec >= 0 && get_seriessession(s, lec))
											{
												lecture_followed_t1 = true;
												break;
											}
										}
									}
									if (!lecture_followed_t1)
									{
										for (int p = 0; p < nb_paths; ++p)
											travels_timeslot_series_uses_path.at(t, s, p) = -get_roomroompath(r2, nb_locations, p); // negative to remember to exclude in T_max
									}
								}
							}
						}
					}
				}

				// 2. flow per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					for (int s = 0; s < nb_series; ++s)
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							double nbp = std::abs(travels_timeslot_series_uses_path.at(t, s, p)) * series_nb_people.at(s);
							if (get_patharc(p, ij))
								travels_timeslot_flow_per_arc.at(t, ij) += nbp;
						}
					}
				}

				// 3. time per arc
				for (int ij = 0; ij < nb_arcs; ++ij)
				{
					travels_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * travels_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
					if (arc_stairs.at(ij))
						travels_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
				}

				// 4. time per series in total
				for (int s = 0; s < nb_series; ++s)
				{
					double time_longest_path = 0.0;
					for (int p = 0; p < nb_paths; ++p)
					{
						if (travels_timeslot_series_uses_path.at(t, s, p) > 0.01)
						{
							double time_current_path = 0.0;
							for (int ij = 0; ij < nb_arcs; ++ij)
							{
								if (get_patharc(p, ij))
									time_current_path += travels_timeslot_time_per_arc.at(t, ij);
							}
							if (time_current_path > time_longest_path)
								time_longest_path = time_current_path;
						}
					}
					travels_timeslot_series_travel_time.at(t, s) = time_longest_path;
				}

				// 5. maximum travel time per timeslot
				for (int s = 0; s < nb_series; ++s)
				{
					if (travels_timeslot_series_travel_time.at(t, s) > new_max_travel_time_changes_ts_roomswap.at(index_ts))
						new_max_travel_time_changes_ts_roomswap.at(index_ts) = travels_timeslot_series_travel_time.at(t, s);
				}

				++index_ts; // go to next timeslot
			}

			// 6. objective travels
			if (_objective_type == objective_type::maximum_over_timeslots)
			{
				index_ts = 0;
				for (auto&& t : _relevant_timeslots)
				{
					if (t < first_timeslot || t >= second_timeslot)
					{
						if (travels_timeslot_max_travel_time.at(t) > objective_travels)
							objective_travels = travels_timeslot_max_travel_time.at(t);
					}
					else
					{
						if (new_max_travel_time_changes_ts_roomswap.at(index_ts) > objective_travels)
							objective_travels = new_max_travel_time_changes_ts_roomswap.at(index_ts);
						++index_ts;
					}
				}
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				index_ts = 0;
				for (auto&& t : _relevant_timeslots)
				{
					if (t < first_timeslot || t >= second_timeslot)
					{
						objective_travels += travels_timeslot_max_travel_time.at(t);
					}
					else
					{
						objective_travels += new_max_travel_time_changes_ts_roomswap.at(index_ts);
						++index_ts;
					}
				}
			}
		}


		// 6. Undo changes
		lec_temp = solution.at(timeslot, room1);
		solution.at(timeslot, room1) = solution.at(timeslot, room2);
		solution.at(timeslot, room2) = lec_temp;


		// 7. recalculate objective value
		double objective_flows = (_alpha * objective_evacuations + (1 - _alpha) * objective_travels);

		return (_penalty_value_constraint_violation * (new_constraint_violations_scheduling_conflicts
			+ new_constraint_violations_correct_room
			+ new_constraint_violations_teacher_working_time
			+ new_constraint_violations_compactness_constraints)
			+ _lambda * new_preference_score + (1 - _lambda) * objective_flows);
	}




	double heuristic::evaluate_incremental(matrix2D<int>& solution, int timeslot1, int timeslot2, int room1, int room2)
	{
		// 1. start from current solution
		new_constraint_violations_scheduling_conflicts = constraint_violations_scheduling_conflicts;
		new_constraint_violations_correct_room = constraint_violations_correct_room;
		new_constraint_violations_teacher_working_time = constraint_violations_correct_room;
		new_constraint_violations_compactness_constraints = constraint_violations_compactness_constraints;
		new_preference_score = preference_score;

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
			if (nb_timeslots_per_day == 6)
			{
				for (int s = 0; s < nb_series; ++s)
				{
					for (int day = 0; day < nb_days; ++day)
					{
						for (int t = 2; t < 3; ++t) // (1-3, 2-4,) 3-5
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
				constraint_violations_compactness_constraints = 0;
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
						for (int t = 2; t < 3; ++t) // (1-3, 2-4,) 3-5
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
				constraint_violations_compactness_constraints = 0;
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
			if (_constraint_preferences && new_preference_score > _constraint_preferences_value)
				new_preference_score += _penalty_value_constraint_violation;

		}



		// 5. calculate flows for relevant timeslots
		// B. FLOWS
		// I. Evacuations
		double objective_evacuations = 0.0;
		if (_alpha > 0.0001)
		{
			evacuations_timeslot_lecture_uses_path.fill(0.0);
			evacuations_timeslot_flow_per_arc.fill(0.0);
			evacuations_timeslot_time_per_arc.fill(0.0);
			evacuations_timeslot_lecture_travel_time.fill(0.0);
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
								evacuations_timeslot_lecture_uses_path.at(t, lec, p) = get_roomroompath(r, nb_locations, p);
						}
					}

					// 2. flow per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						for (int l = 0; l < nb_sessions; ++l)
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								double nbp = evacuations_timeslot_lecture_uses_path.at(t, l, p) * session_nb_people.at(l);
								if (get_patharc(p, ij))
									evacuations_timeslot_flow_per_arc.at(t, ij) += nbp;
							}
						}
					}

					// 3. time per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						evacuations_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * evacuations_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
						if (arc_stairs.at(ij))
							evacuations_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
					}

					// 4. time per lecture in total
					for (int l = 0; l < nb_sessions; ++l)
					{
						double time_longest_path = 0.0;
						for (int p = 0; p < nb_paths; ++p)
						{
							if (evacuations_timeslot_lecture_uses_path.at(t, l, p) > 0.01)
							{
								double time_current_path = 0.0;
								for (int ij = 0; ij < nb_arcs; ++ij)
								{
									if (get_patharc(p, ij))
										time_current_path += evacuations_timeslot_time_per_arc.at(t, ij);
								}
								if (time_current_path > time_longest_path)
									time_longest_path = time_current_path;
							}
						}
						evacuations_timeslot_lecture_travel_time.at(t, l) = time_longest_path;
					}

					// 5. maximum evacuation time per timeslot
					for (int l = 0; l < nb_sessions; ++l)
					{
						if (evacuations_timeslot_lecture_travel_time.at(t, l) > new_max_evac_time_changed_ts.at(index_ts))
							new_max_evac_time_changed_ts.at(index_ts) = evacuations_timeslot_lecture_travel_time.at(t, l);
					}

					++index_ts; // go to next timeslot
				}
			}

			// 6. objective evacuations
			if (_objective_type == objective_type::maximum_over_timeslots)
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
						if (evacuations_timeslot_max_travel_time.at(t) > objective_evacuations)
							objective_evacuations = evacuations_timeslot_max_travel_time.at(t);
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
						objective_evacuations += evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
		}



		// II. Flows between events in consecutive timeslots
		double objective_travels = 0.0;
		if (_alpha < 0.9999)
		{
			travels_timeslot_series_uses_path.fill(0.0);
			travels_timeslot_flow_per_arc.fill(0.0);
			travels_timeslot_time_per_arc.fill(0.0);
			travels_timeslot_series_travel_time.fill(0.0);
			new_max_travel_time_changes_ts.at(0) = 0;
			new_max_travel_time_changes_ts.at(1) = 0;
			new_max_travel_time_changes_ts.at(2) = 0;
			new_max_travel_time_changes_ts.at(3) = 0;

			size_t index_ts = 0;

			for (int t = 0; t < nb_timeslots - 1; ++t)
			{
				if (t == timeslot1 - 1 || t == timeslot1 || t == timeslot2 - 1 || t == timeslot2)
				{
					// 1. who uses which paths
					for (int s = 0; s < nb_series; ++s)
					{
						// a) lecture t and t + 1
						for (int l1 = 0; l1 < nb_sessions; ++l1)
						{
							if (get_seriessession(s, l1))
							{
								for (int r1 = 0; r1 < nb_locations; ++r1)
								{
									if (solution.at(t, r1) == l1) // current timeslot
									{
										for (int l2 = 0; l2 < nb_sessions; ++l2)
										{
											if (get_seriessession(s, l2))
											{
												for (int r2 = 0; r2 < nb_locations; ++r2)
												{
													if (solution.at(t + 1, r2) == l2) // next timeslot
													{
														for (int p = 0; p < nb_paths; ++p)
															travels_timeslot_series_uses_path.at(t, s, p) = get_roomroompath(r1, r2, p);
													}
												}
											}
										}
									}
								}
							}
						}

						// b) lecture t but not t + 1
						for (int l1 = 0; l1 < nb_sessions; ++l1)
						{
							if (get_seriessession(s, l1))
							{
								for (int r1 = 0; r1 < nb_locations; ++r1)
								{
									if (solution.at(t, r1) == l1) // current timeslot
									{
										bool lecture_followed_t2 = false;
										for (int l2 = 0; l2 < nb_sessions; ++l2) // next timeslot
										{
											for (int r2 = 0; r2 < nb_locations; ++r2)
											{
												int lec = solution.at(t + 1, r2);
												if (lec >= 0 && get_seriessession(s, lec))
												{
													lecture_followed_t2 = true;
													break;
												}
											}
										}
										if (!lecture_followed_t2)
										{
											for (int p = 0; p < nb_paths; ++p)
												travels_timeslot_series_uses_path.at(t, s, p) = -get_roomroompath(r1, nb_locations, p);
										}
									}
								}
							}
						}

						// c) lecture t + 1 but not t
						for (int l2 = 0; l2 < nb_sessions; ++l2)
						{
							if (get_seriessession(s, l2))
							{
								for (int r2 = 0; r2 < nb_locations; ++r2)
								{
									if (solution.at(t + 1, r2) == l2) // next timeslot
									{
										bool lecture_followed_t1 = false;
										for (int l1 = 0; l1 < nb_sessions; ++l1) // current timeslot
										{
											for (int r1 = 0; r1 < nb_locations; ++r1)
											{
												int lec = solution.at(t, r1);
												if (lec >= 0 && get_seriessession(s, lec))
												{
													lecture_followed_t1 = true;
													break;
												}
											}
										}
										if (!lecture_followed_t1)
										{
											for (int p = 0; p < nb_paths; ++p)
												travels_timeslot_series_uses_path.at(t, s, p) = -get_roomroompath(r2, nb_locations, p);
										}
									}
								}
							}
						}
					}

					// 2. flow per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						for (int s = 0; s < nb_series; ++s)
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								double nbp = std::abs(travels_timeslot_series_uses_path.at(t, s, p)) * series_nb_people.at(s);
								if (get_patharc(p, ij))
									travels_timeslot_flow_per_arc.at(t, ij) += nbp;
							}
						}
					}

					// 3. time per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						travels_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * travels_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
						if (arc_stairs.at(ij))
							travels_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
					}

					// 4. time per series in total
					for (int s = 0; s < nb_series; ++s)
					{
						double time_longest_path = 0.0;
						for (int p = 0; p < nb_paths; ++p)
						{
							if (travels_timeslot_series_uses_path.at(t, s, p) > 0.01)
							{
								double time_current_path = 0.0;
								for (int ij = 0; ij < nb_arcs; ++ij)
								{
									if (get_patharc(p, ij))
										time_current_path += travels_timeslot_time_per_arc.at(t, ij);
								}
								if (time_current_path > time_longest_path)
									time_longest_path = time_current_path;
							}
						}
						travels_timeslot_series_travel_time.at(t, s) = time_longest_path;
					}

					// 5. maximum travel time per timeslot
					for (int s = 0; s < nb_series; ++s)
					{
						if (travels_timeslot_series_travel_time.at(t, s) > new_max_travel_time_changes_ts.at(index_ts))
							new_max_travel_time_changes_ts.at(index_ts) = travels_timeslot_series_travel_time.at(t, s);
					}

					++index_ts; // go to next timeslot
				}
			}

			// 6. objective travels
			if (_objective_type == objective_type::maximum_over_timeslots)
			{
				index_ts = 0;
				for (auto&& t : _relevant_timeslots)
				{
					if (t == timeslot1 - 1 || t == timeslot1 || t == timeslot2 - 1 || t == timeslot2)
					{
						if (new_max_travel_time_changes_ts.at(index_ts) > objective_travels)
							objective_travels = new_max_travel_time_changes_ts.at(index_ts);
						++index_ts;
					}
					else
					{
						if (travels_timeslot_max_travel_time.at(t) > objective_travels)
							objective_travels = travels_timeslot_max_travel_time.at(t);
					}
				}
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				index_ts = 0;
				for (auto&& t : _relevant_timeslots)
				{
					if (t == timeslot1 - 1 || t == timeslot1 || t == timeslot2 - 1 || t == timeslot2)
					{
						objective_travels += new_max_travel_time_changes_ts.at(index_ts);
						++index_ts;
					}
					else
					{
						objective_travels += travels_timeslot_max_travel_time.at(t);
					}
				}
			}
		}



		// 6. Undo changes
		lec_temp = solution.at(timeslot1, room1);
		solution.at(timeslot1, room1) = solution.at(timeslot2, room2);
		solution.at(timeslot2, room2) = lec_temp;


		// 7. recalculate objective value
		double objective_flows = (_alpha * objective_evacuations + (1 - _alpha) * objective_travels);

		return (_penalty_value_constraint_violation * (new_constraint_violations_scheduling_conflicts
			+ new_constraint_violations_correct_room
			+ new_constraint_violations_teacher_working_time
			+ new_constraint_violations_compactness_constraints)
			+ _lambda * new_preference_score + (1 - _lambda) * objective_flows);
	}




	double heuristic::evaluate_incremental(matrix2D<int>& solution, double current_obj, const std::vector<std::pair<int, int>>& kempe_chain, int timeslot1, int timeslot2)
	{
		// no empty chain
		if (kempe_chain.empty())
			return current_obj;

		// 1. start from current solution
		new_constraint_violations_scheduling_conflicts = constraint_violations_scheduling_conflicts;
		new_constraint_violations_correct_room = constraint_violations_correct_room;
		new_constraint_violations_teacher_working_time = constraint_violations_correct_room;
		new_constraint_violations_compactness_constraints = constraint_violations_compactness_constraints;
		new_preference_score = preference_score;

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
						for (int t = 2; t < 3; ++t) // (1-3, 2-4,) 3-5
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
				constraint_violations_compactness_constraints = 0;
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
									--new_constraint_violations_compactness_constraints;
							}
						}
					}
				}
			}
			else
			{
				// N/A 
				constraint_violations_compactness_constraints = 0;
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
			if (_constraint_preferences && new_preference_score > _constraint_preferences_value)
				new_preference_score += _penalty_value_constraint_violation;
		}



		// 5. calculate flows for relevant timeslots
		// B. FLOWS
		// I. Evacuations
		double objective_evacuations = 0.0;
		if (_alpha > 0.0001)
		{
			evacuations_timeslot_lecture_uses_path.fill(0.0);
			evacuations_timeslot_flow_per_arc.fill(0.0);
			evacuations_timeslot_time_per_arc.fill(0.0);
			evacuations_timeslot_lecture_travel_time.fill(0.0);
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
								evacuations_timeslot_lecture_uses_path.at(t, lec, p) = get_roomroompath(r, nb_locations, p);
						}
					}

					// 2. flow per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						for (int l = 0; l < nb_sessions; ++l)
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								double nbp = evacuations_timeslot_lecture_uses_path.at(t, l, p) * session_nb_people.at(l);
								if (get_patharc(p, ij))
									evacuations_timeslot_flow_per_arc.at(t, ij) += nbp;
							}
						}
					}

					// 3. time per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						evacuations_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * evacuations_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
						if (arc_stairs.at(ij))
							evacuations_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
					}

					// 4. time per lecture in total
					for (int l = 0; l < nb_sessions; ++l)
					{
						double time_longest_path = 0.0;
						for (int p = 0; p < nb_paths; ++p)
						{
							if (evacuations_timeslot_lecture_uses_path.at(t, l, p) > 0.01)
							{
								double time_current_path = 0.0;
								for (int ij = 0; ij < nb_arcs; ++ij)
								{
									if (get_patharc(p, ij))
										time_current_path += evacuations_timeslot_time_per_arc.at(t, ij);
								}
								if (time_current_path > time_longest_path)
									time_longest_path = time_current_path;
							}
						}
						evacuations_timeslot_lecture_travel_time.at(t, l) = time_longest_path;
					}

					// 5. maximum evacuation time per timeslot
					for (int l = 0; l < nb_sessions; ++l)
					{
						if (evacuations_timeslot_lecture_travel_time.at(t, l) > new_max_evac_time_changed_ts.at(index_ts))
							new_max_evac_time_changed_ts.at(index_ts) = evacuations_timeslot_lecture_travel_time.at(t, l);
					}

					++index_ts; // go to next timeslot
				}
			}

			// 6. objective evacuations
			if (_objective_type == objective_type::maximum_over_timeslots)
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
						if (evacuations_timeslot_max_travel_time.at(t) > objective_evacuations)
							objective_evacuations = evacuations_timeslot_max_travel_time.at(t);
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
						objective_evacuations += evacuations_timeslot_max_travel_time.at(t);
					}
				}
			}
		}




		// II. Flows between events in consecutive timeslots
		double objective_travels = 0.0;
		if (_alpha < 0.9999)
		{
			travels_timeslot_series_uses_path.fill(0.0);
			travels_timeslot_flow_per_arc.fill(0.0);
			travels_timeslot_time_per_arc.fill(0.0);
			travels_timeslot_series_travel_time.fill(0.0);
			new_max_travel_time_changes_ts.at(0) = 0;
			new_max_travel_time_changes_ts.at(1) = 0;
			new_max_travel_time_changes_ts.at(2) = 0;
			new_max_travel_time_changes_ts.at(3) = 0;

			size_t index_ts = 0;

			for (int t = 0; t < nb_timeslots - 1; ++t)
			{
				if (t == timeslot1 - 1 || t == timeslot1 || t == timeslot2 - 1 || t == timeslot2)
				{
					// 1. who uses which paths
					for (int s = 0; s < nb_series; ++s)
					{
						// a) lecture t and t + 1
						for (int l1 = 0; l1 < nb_sessions; ++l1)
						{
							if (get_seriessession(s, l1))
							{
								for (int r1 = 0; r1 < nb_locations; ++r1)
								{
									if (solution.at(t, r1) == l1) // current timeslot
									{
										for (int l2 = 0; l2 < nb_sessions; ++l2)
										{
											if (get_seriessession(s, l2))
											{
												for (int r2 = 0; r2 < nb_locations; ++r2)
												{
													if (solution.at(t + 1, r2) == l2) // next timeslot
													{
														for (int p = 0; p < nb_paths; ++p)
															travels_timeslot_series_uses_path.at(t, s, p) = get_roomroompath(r1, r2, p);
													}
												}
											}
										}
									}
								}
							}
						}

						// b) lecture t but not t + 1
						for (int l1 = 0; l1 < nb_sessions; ++l1)
						{
							if (get_seriessession(s, l1))
							{
								for (int r1 = 0; r1 < nb_locations; ++r1)
								{
									if (solution.at(t, r1) == l1) // current timeslot
									{
										bool lecture_followed_t2 = false;
										for (int l2 = 0; l2 < nb_sessions; ++l2) // next timeslot
										{
											for (int r2 = 0; r2 < nb_locations; ++r2)
											{
												int lec = solution.at(t + 1, r2);
												if (lec >= 0 && get_seriessession(s, lec))
												{
													lecture_followed_t2 = true;
													break;
												}
											}
										}
										if (!lecture_followed_t2)
										{
											for (int p = 0; p < nb_paths; ++p)
												travels_timeslot_series_uses_path.at(t, s, p) = -get_roomroompath(r1, nb_locations, p);
										}
									}
								}
							}
						}

						// c) lecture t + 1 but not t
						for (int l2 = 0; l2 < nb_sessions; ++l2)
						{
							if (get_seriessession(s, l2))
							{
								for (int r2 = 0; r2 < nb_locations; ++r2)
								{
									if (solution.at(t + 1, r2) == l2) // next timeslot
									{
										bool lecture_followed_t1 = false;
										for (int l1 = 0; l1 < nb_sessions; ++l1) // current timeslot
										{
											for (int r1 = 0; r1 < nb_locations; ++r1)
											{
												int lec = solution.at(t, r1);
												if (lec >= 0 && get_seriessession(s, lec))
												{
													lecture_followed_t1 = true;
													break;
												}
											}
										}
										if (!lecture_followed_t1)
										{
											for (int p = 0; p < nb_paths; ++p)
												travels_timeslot_series_uses_path.at(t, s, p) = -get_roomroompath(r2, nb_locations, p);
										}
									}
								}
							}
						}
					}

					// 2. flow per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						for (int s = 0; s < nb_series; ++s)
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								double nbp = std::abs(travels_timeslot_series_uses_path.at(t, s, p)) * series_nb_people.at(s);
								if (get_patharc(p, ij))
									travels_timeslot_flow_per_arc.at(t, ij) += nbp;
							}
						}
					}

					// 3. time per arc
					for (int ij = 0; ij < nb_arcs; ++ij)
					{
						travels_timeslot_time_per_arc.at(t, ij) = arc_length.at(ij) / walking_alpha * travels_timeslot_flow_per_arc.at(t, ij) / arc_area.at(ij) + arc_length.at(ij) / v_max;
						if (arc_stairs.at(ij))
							travels_timeslot_time_per_arc.at(t, ij) *= speed_correction_stairs;
					}

					// 4. time per series in total
					for (int s = 0; s < nb_series; ++s)
					{
						double time_longest_path = 0.0;
						for (int p = 0; p < nb_paths; ++p)
						{
							if (travels_timeslot_series_uses_path.at(t, s, p) > 0.01)
							{
								double time_current_path = 0.0;
								for (int ij = 0; ij < nb_arcs; ++ij)
								{
									if (get_patharc(p, ij))
										time_current_path += travels_timeslot_time_per_arc.at(t, ij);
								}
								if (time_current_path > time_longest_path)
									time_longest_path = time_current_path;
							}
						}
						travels_timeslot_series_travel_time.at(t, s) = time_longest_path;
					}

					// 5. maximum travel time per timeslot
					for (int s = 0; s < nb_series; ++s)
					{
						if (travels_timeslot_series_travel_time.at(t, s) > new_max_travel_time_changes_ts.at(index_ts))
							new_max_travel_time_changes_ts.at(index_ts) = travels_timeslot_series_travel_time.at(t, s);
					}

					++index_ts; // go to next timeslot
				}
			}

			// 6. objective travels
			if (_objective_type == objective_type::maximum_over_timeslots)
			{
				index_ts = 0;
				for (auto&& t : _relevant_timeslots)
				{
					if (t == timeslot1 - 1 || t == timeslot1 || t == timeslot2 - 1 || t == timeslot2)
					{
						if (new_max_travel_time_changes_ts.at(index_ts) > objective_travels)
							objective_travels = new_max_travel_time_changes_ts.at(index_ts);
						++index_ts;
					}
					else
					{
						if (travels_timeslot_max_travel_time.at(t) > objective_travels)
							objective_travels = travels_timeslot_max_travel_time.at(t);
					}
				}
			}
			else // _objective_type == objective_type::sum_over_timeslots
			{
				index_ts = 0;
				for (auto&& t : _relevant_timeslots)
				{
					if (t == timeslot1 - 1 || t == timeslot1 || t == timeslot2 - 1 || t == timeslot2)
					{
						objective_travels += new_max_travel_time_changes_ts.at(index_ts);
						++index_ts;
					}
					else
					{
						objective_travels += travels_timeslot_max_travel_time.at(t);
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
		double objective_flows = (_alpha * objective_evacuations + (1 - _alpha) * objective_travels);

		return (_penalty_value_constraint_violation * (new_constraint_violations_scheduling_conflicts
			+ new_constraint_violations_correct_room
			+ new_constraint_violations_teacher_working_time
			+ new_constraint_violations_compactness_constraints)
			+ _lambda * new_preference_score + (1 - _lambda) * objective_flows);
	}




}