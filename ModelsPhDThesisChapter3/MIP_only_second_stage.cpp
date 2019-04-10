#include "MIP_only_second_stage.h"
#include "timetable_data.h"
#include "building_data.h"
#include "initial_solution.h"
#include "logger.h"

#include <stdexcept>
#include <iostream>
#include <memory>


namespace alg
{

	void MIP_only_second_stage::run_algorithm(bool constraint_on_evacuation_time, double value)
	{
		auto start_time = std::chrono::system_clock::now();

		initialize_cplex();
		build_problem();
		if (constraint_on_evacuation_time)
			add_constraint_maximum_evacuation_time(value);
		solve_problem();

		computation_time = std::chrono::system_clock::now() - start_time;

		write_output();

		clear_cplex();
	}


	void MIP_only_second_stage::initialize_cplex()
	{
		int status = 0;

		// Open the cplex environment
		env = CPXopenCPLEX(&status);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::initialize_cplex(). \nCouldn't open CPLEX. \nReason: " + std::string(error_text));
		}

		// Set the output to screen on/off
		status = CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::initialize_cplex(). \nCouldn't set the output to screen off. \nReason: " + std::string(error_text));
		}

		// Set tolerance gap
		status = CPXsetdblparam(env, CPX_PARAM_EPGAP, _optimality_tolerance);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::initialize_cplex(). \nCouldn't set the tolerance gap. \nReason: " + std::string(error_text));
		}

		// Time limit for the MIP
		status = CPXsetdblparam(env, CPX_PARAM_TILIM, _time_limit);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::initialize_cplex(). \nCouldn't set the time limit. \nReason: " + std::string(error_text));
		}
	}


	void MIP_only_second_stage::build_problem()
	{
		constexpr int Big_M = 10000;
		int status = 0;
		double obj[1];						// Objective function
		double lb[1];						// Lower bound variables
		double ub[1];						// Upper bound variables
		double rhs[1];						// Right-hand side constraints
		char *colname[1];					// Variable names
		char *rowname[1];					// Constraint names
		char sense[1];						// Sign of constraint
		int matbeg[1];						// Begin position of the constraint
		std::unique_ptr<int[]> matind;		// Position of each element in constraint matrix
		std::unique_ptr<double[]> matval;	// Value of each element in constraint matrix
		char type[1];						// Type of variable (integer, binary, fractional)
		int f{ 0 };							// To calculate number of nonzero coefficients in each constraint


											// Create arrays
		matind = std::make_unique<int[]>(10000000);
		matval = std::make_unique<double[]>(10000000);

		// Create the problem
		problem = CPXcreateprob(env, &status, "problem_second_stage");
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't create the CPLEX problem. \nReason: " + std::string(error_text));
		}

		// Problem is minimization
		CPXchgobjsen(env, problem, CPX_MIN);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't set the problem type to minimization. \nReason: " + std::string(error_text));
		}

		// VARIABLES
		// Add the W_lc variables
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int c = 0; c < nb_locations; ++c)
			{
				std::string name = "w_" + std::to_string(l + 1) + "_" + std::to_string(c + 1);
				colname[0] = const_cast<char*>(name.c_str());

				obj[0] = 0;

				lb[0] = 0;
				ub[0] = 1;
				type[0] = 'B';

				status = CPXnewcols(env, problem, 1, obj, lb, ub, type, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Add the U(travel)_tsp variables
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int p = 0; p < nb_paths; ++p)
				{
					std::string name = "U_travel_" + std::to_string(t + 1) + "_" + std::to_string(s + 1) + "_" + std::to_string(p + 1);
					colname[0] = const_cast<char*>(name.c_str());

					obj[0] = 0;
					lb[0] = 0;

					status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}

		// Add the T(travel_arc)_tij variables
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int ij = 0; ij < nb_arcs; ++ij)
			{
				std::string name = "T_travel_arc_" + std::to_string(t + 1) + "_" + std::to_string(ij + 1);
				colname[0] = const_cast<char*>(name.c_str());

				obj[0] = 0;
				lb[0] = 0;

				status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Add the T(travel)_tsp variables
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int p = 0; p < nb_paths; ++p)
				{
					std::string name = "T_travel_total_" + std::to_string(t + 1) + "_" + std::to_string(s + 1) + "_" + std::to_string(p + 1);
					colname[0] = const_cast<char*>(name.c_str());

					obj[0] = 0;
					lb[0] = 0;

					status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}

		// Add the T(travel)_t,max variables
		for (int t = 0; t < nb_timeslots; ++t)
		{
			std::string name = "T_travel_max_" + std::to_string(t + 1);
			colname[0] = const_cast<char*>(name.c_str());

			if (_objective_type == objective_type::maximum_over_timeslots)
				obj[0] = 0;
			else
				obj[0] = (1 - _alpha);
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the U(evac)_tlp variables
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int p = 0; p < nb_paths; ++p)
				{
					std::string name = "U_evac_" + std::to_string(t + 1) + "_" + std::to_string(l + 1) + "_" + std::to_string(p + 1);
					colname[0] = const_cast<char*>(name.c_str());

					obj[0] = 0;
					lb[0] = 0;

					status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}

		// Add the T(evac_arc)_tij variables
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int ij = 0; ij < nb_arcs; ++ij)
			{
				std::string name = "T_evac_arc_" + std::to_string(t + 1) + "_" + std::to_string(ij + 1);
				colname[0] = const_cast<char*>(name.c_str());

				obj[0] = 0;
				lb[0] = 0;

				status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Add the T(evac)_tlp variables
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int p = 0; p < nb_paths; ++p)
				{
					std::string name = "T_evac_total_" + std::to_string(t + 1) + "_" + std::to_string(l + 1) + "_" + std::to_string(p + 1);
					colname[0] = const_cast<char*>(name.c_str());

					obj[0] = 0;
					lb[0] = 0;

					status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}

		// Add the T(evac)_t,max variables
		for (int t = 0; t < nb_timeslots; ++t)
		{
			std::string name = "T_evac_max_" + std::to_string(t + 1);
			colname[0] = const_cast<char*>(name.c_str());

			if (_objective_type == objective_type::maximum_over_timeslots)
				obj[0] = 0;
			else
				obj[0] = _alpha;
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(travel)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_travel_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = (1 - _alpha);
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(evac)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_evac_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = _alpha;
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}


		// CONSTRAINTS
		// TIMETABLE
		// Constraint set 1: Each Lecture should be scheduled and at an appropriate location
		for (int l = 0; l < nb_sessions; ++l)
		{
			sense[0] = 'E';
			rhs[0] = 1;

			std::string name = "Lecture_" + std::to_string(l + 1) + "_scheduled";
			rowname[0] = const_cast<char*>(name.c_str());

			matbeg[0] = 0;
			f = 0;

			for (int c = 0; c < nb_locations; ++c)
			{
				if (get_sessionlocationpossible(l, c))
				{
					matind[f] = l * nb_locations + c;
					matval[f] = 1;
					++f;
				}
			}

			status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
			}
		}


		// Constraint set 2: No room overlaps
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int c = 0; c < nb_locations; ++c)
			{
				sense[0] = 'L';
				rhs[0] = 1;

				std::string name = "Room_" + std::to_string(c + 1) + "_max_one_lecture_at_time_" + std::to_string(t + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_sessionlocationpossible(l, c) && initial_solution_lecture_timing[l] == t)
					{
						matind[f] = l * nb_locations + c;
						matval[f] = 1;
						++f;
					}
				}

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}


		// FLOWS BETWEEN EVENTS IN CONSECUTIVE TIMESLOTS
		// Constraint set 8: U_tsp constraints 
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && initial_solution_lecture_timing[l] == t) // only lectures planned in this timeslot
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && initial_solution_lecture_timing[m] == t + 1) // lectures planned in the next timeslot
							{
								for (int c = 0; c < nb_locations; ++c)
								{
									if (get_sessionlocationpossible(l, c))
									{
										for (int d = 0; d < nb_locations; ++d)
										{
											if (get_sessionlocationpossible(m, d))
											{
												for (int p = 0; p < nb_paths; ++p)
												{
													if (get_roomroompath(c, d, p))
													{
														sense[0] = 'L';
														rhs[0] = get_roomroompath(c, d, p);

														std::string name = "TRAVELS_path_" + std::to_string(p + 1) + "_used_by_series_" + std::to_string(s + 1) + "_at_time_" + std::to_string(t + 1);
														rowname[0] = const_cast<char*>(name.c_str());

														matbeg[0] = 0;
														f = 0;

														// w_lc
														matind[f] = l * nb_locations + c;
														matval[f] = get_roomroompath(c, d, p);
														++f;

														// w_md
														matind[f] = m * nb_locations + d;
														matval[f] = get_roomroompath(c, d, p);
														++f;

														// U_tsp
														matind[f] = nb_sessions * nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
														matval[f] = -1;
														++f;


														status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 8bis: U_tsp constraints no lecture time t+1
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && initial_solution_lecture_timing[l] == t)
					{
						for (int c = 0; c < nb_locations; ++c)
						{
							if (get_sessionlocationpossible(l, c))
							{
								for (int p = 0; p < nb_paths; ++p)
								{
									if (get_roomroompath(c, nb_locations, p))
									{
										sense[0] = 'L';
										rhs[0] = 0;

										std::string name = "TRAVELS_path_" + std::to_string(p + 1) + "_used_by_series_" + std::to_string(s + 1) + "_at_time_" + std::to_string(t + 1);
										rowname[0] = const_cast<char*>(name.c_str());

										matbeg[0] = 0;
										f = 0;

										// w_lc
										matind[f] = l * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of w_md
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m) && initial_solution_lecture_timing[m] == t + 1)
											{
												for (int d = 0; d < nb_locations; ++d)
												{
													if (get_sessionlocationpossible(m, d))
													{
														// x_m,t+1,d
														matind[f] = m * nb_locations + d;
														matval[f] = -get_roomroompath(c, nb_locations, p);
														++f;
													}
												}
											}
										}

										// U_tsp
										matind[f] = nb_sessions * nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
										matval[f] = -1;
										++f;

										status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
										}
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 8tres: U_tsp constraints no lecture time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && initial_solution_lecture_timing[l] == t + 1)
					{
						for (int c = 0; c < nb_locations; ++c)
						{
							if (get_sessionlocationpossible(l, c))
							{
								for (int p = 0; p < nb_paths; ++p)
								{
									if (get_roomroompath(c, nb_locations, p))
									{
										sense[0] = 'L';
										rhs[0] = 0;

										std::string name = "TRAVELS_path_" + std::to_string(p + 1) + "_used_by_series_" + std::to_string(s + 1) + "_at_time_" + std::to_string(t + 1);
										rowname[0] = const_cast<char*>(name.c_str());

										matbeg[0] = 0;
										f = 0;

										// w_lc
										matind[f] = l * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of w_md
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m) && initial_solution_lecture_timing[m] == t)
											{
												for (int d = 0; d < nb_locations; ++d)
												{
													if (get_sessionlocationpossible(m, d))
													{
														matind[f] = m * nb_locations + d;
														matval[f] = -get_roomroompath(c, nb_locations, p);
														++f;
													}
												}
											}
										}

										// U_tsp
										matind[f] = nb_sessions * nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
										matval[f] = -1;
										++f;

										status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
										}
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 9: maximum crowd flow per arc
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int ij = 0; ij < nb_arcs; ++ij)
			{
				sense[0] = 'G';
				rhs[0] = -F_max;

				std::string name = "TRAVELS_maximum_flow_time_" + std::to_string(t + 1) + "_arc_" + std::to_string(ij + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				for (int p = 0; p < nb_paths; ++p)
				{
					for (int s = 0; s < nb_series; ++s)
					{
						matind[f] = nb_sessions * nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
						matval[f] = -series_nb_people[s] * get_patharc(p, ij);;
						++f;
					}
				}
				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 10: Travel time through arc ij at time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int ij = 0; ij < nb_arcs; ++ij)
			{
				sense[0] = 'E';
				rhs[0] = (double)arc_length[ij] / v_max;

				std::string name = "TRAVELS_travel_time_arc_t_" + std::to_string(t + 1) + "_arc_" + std::to_string(ij + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				// U_tsp variables
				for (int p = 0; p < nb_paths; ++p)
				{
					for (int s = 0; s < nb_series; ++s)
					{
						matind[f] = nb_sessions * nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
						matval[f] = -series_nb_people[s] * get_patharc(p, ij) * (double)arc_length[ij] / (walking_alpha*arc_area[ij]);
						if (arc_stairs[ij])
							matval[f] *= speed_correction_stairs;
						++f;
					}
				}

				// T(arc)_tij variable
				matind[f] = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + t * nb_arcs + ij;
				matval[f] = 1;
				++f;

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 11: Travel time for series s at time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && initial_solution_lecture_timing[l] == t)
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && initial_solution_lecture_timing[m] == t + 1)
							{
								for (int c = 0; c < nb_locations; ++c)
								{
									if (get_sessionlocationpossible(l, c))
									{
										for (int d = 0; d < nb_locations; ++d)
										{
											if (get_sessionlocationpossible(m, d))
											{
												for (int p = 0; p < nb_paths; ++p)
												{
													if (get_roomroompath(c, d, p) > 0)
													{
														sense[0] = 'L';
														rhs[0] = 2 * Big_M;

														std::string name = "TRAVELS_travel_time_series_" + std::to_string(s + 1) + "_time_" + std::to_string(t + 1) + "_path_" + std::to_string(p + 1);
														rowname[0] = const_cast<char*>(name.c_str());

														matbeg[0] = 0;
														f = 0;

														// w_lc
														matind[f] = l * nb_locations + c;
														matval[f] = Big_M;
														++f;

														// w_md
														matind[f] = m * nb_locations + d;
														matval[f] = Big_M;
														++f;

														// sum of T_arc_t,ij
														for (int ij = 0; ij < nb_arcs; ++ij)
														{
															matind[f] = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + t * nb_arcs + ij;
															matval[f] = -get_patharc(p, ij);
															++f;
														}

														// T_tot_tsp
														matind[f] = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + t * nb_series*nb_paths + s * nb_paths + p;
														matval[f] = 1;
														++f;

														status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 11bis: Travel time for series s at time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && initial_solution_lecture_timing[l] == t)
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && initial_solution_lecture_timing[m] == t + 1)
							{
								for (int c = 0; c < nb_locations; ++c)
								{
									if (get_sessionlocationpossible(l, c))
									{
										for (int d = 0; d < nb_locations; ++d)
										{
											if (get_sessionlocationpossible(m, d))
											{
												for (int p = 0; p < nb_paths; ++p)
												{
													if (get_roomroompath(c, d, p) > 0)
													{
														sense[0] = 'L';
														rhs[0] = 2 * Big_M;

														std::string name = "TRAVELS_travel_time_series_" + std::to_string(s + 1) + "_time_" + std::to_string(t + 1) + "_path_" + std::to_string(p + 1);
														rowname[0] = const_cast<char*>(name.c_str());

														matbeg[0] = 0;
														f = 0;

														// w_lc
														matind[f] = l * nb_locations + c;
														matval[f] = Big_M;
														++f;

														// w_md
														matind[f] = m * nb_locations + d;
														matval[f] = Big_M;
														++f;

														// sum of T_arc_t,ij
														for (int ij = 0; ij < nb_arcs; ++ij)
														{
															matind[f] = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + t * nb_arcs + ij;
															matval[f] = get_patharc(p, ij);
															++f;
														}

														// T_tot_tsp
														matind[f] = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + t * nb_series*nb_paths + s * nb_paths + p;
														matval[f] = -1;
														++f;

														status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
														}

													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 12: Maximum travel time timeslot t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int p = 0; p < nb_paths; ++p)
				{
					sense[0] = 'L';
					rhs[0] = 0;

					std::string name = "TRAVELS_travel_time_t_" + std::to_string(t + 1) + "_s_" + std::to_string(s + 1) + "_p_" + std::to_string(p + 1);
					rowname[0] = const_cast<char*>(name.c_str());

					matbeg[0] = 0;
					f = 0;

					// T(travel)_tsp
					matind[f] = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + t * nb_series*nb_paths + s * nb_paths + p;
					matval[f] = 1;
					++f;

					// T(travel)_t,max
					matind[f] = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series*nb_paths + t;
					matval[f] = -1;
					++f;

					status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}


		// EVACUATIONS
		const int nb_variables_first_part = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series*nb_paths + nb_timeslots;

		// Constraint set 13: U_tlp constraints 
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (initial_solution_lecture_timing[l] == t) // only lectures planned in this timeslot
				{
					for (int c = 0; c < nb_locations; ++c)
					{
						if (get_sessionlocationpossible(l, c))
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								if (get_roomroompath(c, nb_locations, p))
								{
									sense[0] = 'L';
									rhs[0] = 0;

									std::string name = "EVACS_path_" + std::to_string(p + 1) + "_used_by_lecture_" + std::to_string(l + 1) + "_at_time_" + std::to_string(t + 1);
									rowname[0] = const_cast<char*>(name.c_str());

									matbeg[0] = 0;
									f = 0;

									// w_lc
									matind[f] = l * nb_locations + c;
									matval[f] = get_roomroompath(c, nb_locations, p);
									++f;

									// U_tlp
									matind[f] = nb_variables_first_part + t * nb_sessions*nb_paths + l * nb_paths + p;
									matval[f] = -1;
									++f;


									status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 14: maximum crowd flow per arc
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int ij = 0; ij < nb_arcs; ++ij)
			{
				sense[0] = 'G';
				rhs[0] = -F_max;

				std::string name = "EVACS_maximum_flow_time_" + std::to_string(t + 1) + "_arc_" + std::to_string(ij + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				for (int p = 0; p < nb_paths; ++p)
				{
					for (int l = 0; l < nb_sessions; ++l)
					{
						matind[f] = nb_variables_first_part + t * nb_sessions*nb_paths + l * nb_paths + p;
						matval[f] = -session_nb_people[l] * get_patharc(p, ij);
						++f;
					}
				}

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 15: Evacuation time through arc ij at time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int ij = 0; ij < nb_arcs; ++ij)
			{
				sense[0] = 'E';
				rhs[0] = (double)arc_length[ij] / v_max;

				std::string name = "EVACS_travel_time_arc_t_" + std::to_string(t + 1) + "_arc_" + std::to_string(ij + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				// U_tlp variables
				for (int p = 0; p < nb_paths; ++p)
				{
					for (int l = 0; l < nb_sessions; ++l)
					{
						matind[f] = nb_variables_first_part + t * nb_sessions*nb_paths + l * nb_paths + p;
						matval[f] = -session_nb_people[l] * get_patharc(p, ij) * (double)arc_length[ij] / (walking_alpha*arc_area[ij]);
						if (arc_stairs[ij])
							matval[f] *= speed_correction_stairs;
						++f;
					}
				}

				// T(arc)_tij variable
				matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + t * nb_arcs + ij;
				matval[f] = 1;
				++f;

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 16: Evacuation time for lecture l at time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (initial_solution_lecture_timing[l] == t)
				{
					for (int c = 0; c < nb_locations; ++c)
					{
						if (get_sessionlocationpossible(l, c))
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								if (get_roomroompath(c, nb_locations, p))
								{
									sense[0] = 'L';
									rhs[0] = Big_M;

									std::string name = "EVACS_travel_time_lecture_" + std::to_string(l + 1) + "_time_" + std::to_string(t + 1) + "_path_" + std::to_string(p + 1);
									rowname[0] = const_cast<char*>(name.c_str());

									matbeg[0] = 0;
									f = 0;

									// w_lc
									matind[f] = l * nb_locations + c;
									matval[f] = Big_M;
									++f;

									// sum of T_arc_t,ij
									for (int ij = 0; ij < nb_arcs; ++ij)
									{
										matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + t * nb_arcs + ij;
										matval[f] = -get_patharc(p, ij);
										++f;
									}

									// T_tot_tsp
									matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + t * nb_sessions*nb_paths + l * nb_paths + p;
									matval[f] = 1;
									++f;

									status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 16bis: Evacuation time for lecture l at time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (initial_solution_lecture_timing[l] == t)
				{
					for (int c = 0; c < nb_locations; ++c)
					{
						if (get_sessionlocationpossible(l, c))
						{
							for (int p = 0; p < nb_paths; ++p)
							{
								if (get_roomroompath(c, nb_locations, p))
								{
									sense[0] = 'L';
									rhs[0] = Big_M;

									std::string name = "EVACS_travel_time_lecture_" + std::to_string(l + 1) + "_time_" + std::to_string(t + 1) + "_path_" + std::to_string(p + 1);
									rowname[0] = const_cast<char*>(name.c_str());

									matbeg[0] = 0;
									f = 0;

									// w_lc
									matind[f] = l * nb_locations + c;
									matval[f] = Big_M;
									++f;

									// sum of T_arc_t,ij
									for (int ij = 0; ij < nb_arcs; ++ij)
									{
										matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + t * nb_arcs + ij;
										matval[f] = get_patharc(p, ij);
										++f;
									}

									// T_tot_tsp
									matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + t * nb_sessions*nb_paths + l * nb_paths + p;
									matval[f] = -1;
									++f;

									status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 17: Maximum evacuation time timeslot t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int p = 0; p < nb_paths; ++p)
				{
					sense[0] = 'L';
					rhs[0] = 0;

					std::string name = "EVACS_travel_time_t_" + std::to_string(t + 1) + "_l_" + std::to_string(l + 1) + "_p_" + std::to_string(p + 1);
					rowname[0] = const_cast<char*>(name.c_str());

					matbeg[0] = 0;
					f = 0;

					// T(evac)_tlp
					matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + t * nb_sessions*nb_paths + l * nb_paths + p;
					matval[f] = 1;
					++f;

					// T(travel)_t,max
					matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions*nb_paths + t;
					matval[f] = -1;
					++f;

					status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}


		// MAXIMUM
		// Constraint set 18: Maximal travel time
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int s = 0; s < nb_series; ++s)
				{
					for (int p = 0; p < nb_paths; ++p)
					{
						sense[0] = 'G';
						rhs[0] = 0;

						std::string name = "TRAVELS_maximum_travel_time_t_" + std::to_string(t + 1) + "_s_" + std::to_string(s + 1) + "_p_" + std::to_string(p + 1);
						rowname[0] = const_cast<char*>(name.c_str());

						matbeg[0] = 0;
						f = 0;

						// T(total)_tsp variable
						matind[f] = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + t * nb_series*nb_paths + s * nb_paths + p;
						matval[f] = -1;
						++f;

						// Theta (Tmax) variable
						matind[f] = CPXgetnumcols(env, problem) - 2;
						matval[f] = 1;
						++f;

						status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}

		// Constraint set 19: Maximal evacuation time
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					for (int p = 0; p < nb_paths; ++p)
					{
						sense[0] = 'G';
						rhs[0] = 0;

						std::string name = "EVACS_maximum_travel_time_t_" + std::to_string(t + 1) + "_l_" + std::to_string(l + 1) + "_p_" + std::to_string(p + 1);
						rowname[0] = const_cast<char*>(name.c_str());

						matbeg[0] = 0;
						f = 0;

						// T(total)_tsp variable
						matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + t * nb_sessions*nb_paths + l * nb_paths + p;
						matval[f] = -1;
						++f;

						// Theta (Tmax) variable
						matind[f] = CPXgetnumcols(env, problem) - 1;
						matval[f] = 1;
						++f;

						status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}


		// Write to file
		status = CPXwriteprob(env, problem, "MIP_model_second_stage.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::build_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}

		// Get info
		numcols_problem = CPXgetnumcols(env, problem);
		numrows_problem = CPXgetnumrows(env, problem);



		// Initialize vectors
		solution_lecture_room.reserve(nb_sessions*nb_locations);
		for (int l = 0; l < nb_sessions; ++l)
		{
			solution_lecture_room.push_back(-1);
		}

		solution_travel_times.reserve(nb_timeslots);
		for (int t = 0; t < nb_timeslots; ++t)
		{
			solution_travel_times.push_back(0.0);
		}

		solution_evacuation_times.reserve(nb_timeslots);
		for (int t = 0; t < nb_timeslots; ++t)
		{
			solution_evacuation_times.push_back(0.0);
		}
	}


	void MIP_only_second_stage::solve_problem()
	{
		int status = 0;
		int solstat;
		std::unique_ptr<double[]> solution_CPLEX = std::make_unique<double[]>(numcols_problem);


		// Optimize the problem
		std::cout << "\n\nCPLEX is solving the second stage MIP for instance " << instance_name_timetable << " + " << instance_name_building << " ... ";
		status = CPXmipopt(env, problem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
		}

		// Get the solution
		status = CPXsolution(env, problem, &solstat, &objective_value, solution_CPLEX.get(), NULL, NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
		}


		// Check the solution status
		if (solstat == CPXMIP_OPTIMAL) {
			solution_exists = true;
			solution_info = "Optimal solution found";
		}
		else if (solstat == CPXMIP_OPTIMAL_TOL) {
			solution_exists = true;
			solution_info = "Solution found within tolerance limit";
		}
		else if (solstat == CPXMIP_TIME_LIM_FEAS) {
			solution_exists = true;
			solution_info = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_FEAS) {
			solution_exists = true;
			solution_info = "Tree memory limit exceeded";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_exists = false;
			solution_info = "Problem is infeasible";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_exists = false;
			solution_info = "Problem is infeasible";
		}
		else if (solstat == CPXMIP_UNBOUNDED) {
			solution_exists = false;
			solution_info = "Problem is unbounded";
		}
		else if (solstat == CPXMIP_INForUNBD) {
			solution_exists = false;
			solution_info = "Problem is infeasible or unbounded";
		}
		else if (solstat == CPXMIP_TIME_LIM_INFEAS) {
			solution_exists = false;
			solution_info = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_INFEAS) {
			solution_exists = false;
			solution_info = "Tree memory limit exceeded";
		}
		else {
			solution_exists = false;
			solution_info = "Other reason for failure";
		}

		std::cout << "\nCPLEX has finished: " << solution_info << "\n";



		/////
		/*{
		const int index_evacs = nb_sessions * nb_locations	// w_lc
		+ nb_timeslots * nb_series * nb_paths			// U_travel_tsp
		+ nb_timeslots * nb_arcs						// T_travel_arc_tij
		+ nb_timeslots * nb_series * nb_paths			// T_travel_total_tsp
		+ nb_timeslots;									// T_travel_max_t

		//+nb_timeslots * nb_sessions * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions * nb_paths;

		std::cout << "\n\nObjective value = " << objective_value << "\n";
		for (int l = 0; l < nb_sessions; ++l) {
		for (int c = 0; c < nb_locations; ++c) {
		if (solution_CPLEX[l*nb_locations + c] > 0.99)
		std::cout << "\nLecture " << l + 1 << " assigned to room " << c + 1;
		}
		}

		for (int t = 0; t < nb_timeslots; ++t) {
		for (int l = 0; l < nb_sessions; ++l) {
		for (int p = 0; p < nb_paths; ++p) {
		if (solution_CPLEX[index_evacs + t * nb_sessions*nb_paths + l * nb_paths + p] > 0.01)
		std::cout << "\nU_evac_t_" << t + 1 << "_l_" << l + 1 << "_p_" << p + 1 << " = " << solution_CPLEX[index_evacs + t * nb_sessions*nb_paths + l * nb_paths + p];
		}
		}
		}

		for (int t = 0; t < nb_timeslots; ++t) {
		for (int ij = 0; ij < nb_arcs; ++ij) {
		if (solution_CPLEX[index_evacs + nb_timeslots * nb_sessions*nb_paths + t * nb_arcs + ij] > 0.01)
		std::cout << "\nT_evac_arc_t" << t + 1 << "_ij_" << ij + 1 << " = " << solution_CPLEX[index_evacs + nb_timeslots*nb_sessions*nb_paths + t*nb_arcs + ij];
		}
		}

		for (int t = 0; t < nb_timeslots; ++t) {
		for (int l = 0; l < nb_sessions; ++l) {
		for (int p = 0; p < nb_paths; ++p) {
		if (solution_CPLEX[index_evacs + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + t * nb_sessions*nb_paths + l * nb_paths + p] > 0.01)
		std::cout << "\nT_evac_total_t_" << t + 1 << "_l_" << l + 1 << "_p_" << p + 1 << " = " << solution_CPLEX[index_evacs + nb_timeslots*nb_sessions*nb_paths + nb_timeslots*nb_arcs + t*nb_sessions*nb_paths + l*nb_paths + p];
		}
		}
		}

		for (int t = 0; t < nb_timeslots; ++t) {
		if (solution_CPLEX[index_evacs + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions*nb_paths + t] > 0.01)
		std::cout << "\nT_evac_total_max_t_" << t + 1 << " = " << solution_CPLEX[index_evacs + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions*nb_paths + t];
		}
		}*/

		/////


		if (solution_exists)
		{
			// 1. solution: assignment of lectures to rooms
			for (int l = 0; l < nb_sessions; ++l)
			{
				solution_lecture_room.at(l) = -1;
				for (int r = 0; r < nb_locations; ++r)
				{
					if (solution_CPLEX[l*nb_locations + r] > 0.99)
					{
						solution_lecture_room.at(l) = r;
						break;
					}
				}
			}

			// 2. travel times
			{
				const int index_travels = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					double time = solution_CPLEX[index_travels + t];
					solution_travel_times.at(t) = time;
				}
			}

			// 3. evacuation times
			{
				const int index_evacs = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths
					+ nb_timeslots + nb_timeslots * nb_sessions * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions * nb_paths;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					double time = solution_CPLEX[index_evacs + t];
					solution_evacuation_times.at(t) = time;
				}
			}
		}
	}


	void MIP_only_second_stage::clear_cplex()
	{
		int status = 0;

		// Clear vectors
		solution_lecture_room.clear();
		solution_travel_times.clear();
		solution_evacuation_times.clear();

		// Free the problem
		status = CPXfreeprob(env, &problem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::clear_cplex(). \nCouldn't free the master problem. \nReason: " + std::string(error_text));
		}

		// Close the cplex environment
		status = CPXcloseCPLEX(&env);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::clear_cplex(). \nCouldn't close CPLEX environment. \nReason: " + std::string(error_text));
		}
	}



	///////////////////////

	// EXHAUSTIVE SEARCH
	void MIP_only_second_stage::run_exhaustive_search()
	{
		std::string logger_text;

		std::cout << "\n\nStarting Exhaustive Search ...";
		logger_text = "Starting Exhaustive Search ...";
		_logger << logger::log_type::INFORMATION << logger_text;
		_logger << logger::log_type::INFORMATION;

		// build cplex
		initialize_cplex();
		build_problem();

		// reset counter
		exhaustive_search_solutions_number = 0;

		// clear room assignments
		for (int e = 0; e < nb_sessions; ++e)
			solution_lecture_room.at(e) = -1;

		// if evacuations only
		if (_alpha > 0.99)
		{
			// every timeslot is independent of all others
			for (int t = 0; t < nb_timeslots; ++t)
			{
				generate_all_possible_solutions_independently(t, 0);
			}
		}
		// travels between consecutive timeslots only
		else
		{
			generate_all_possible_solutions(0);
		}

		// If done
		std::cout << "\n\nExhaustive search finished";

		// clear cplex
		clear_cplex();
	}


	// RECURSIVE FUNCTION TO GENERATE ALL SOLUTIONS (for travels)
	void MIP_only_second_stage::generate_all_possible_solutions(int current_event)
	{
		// solution has been generated entirely
		if (current_event >= nb_sessions)
		{
			fix_solution(solution_lecture_room);
			solve_problem();
			CPXdelrows(env, problem, CPXgetnumrows(env, problem) - nb_sessions, CPXgetnumrows(env, problem) - 1);

			++exhaustive_search_solutions_number;
			std::string text;
			text = "Solution," + std::to_string(exhaustive_search_solutions_number) + ",Objective_value," + std::to_string(objective_value);
			_logger << logger::log_type::INFORMATION << text;
		}
		else
		{
			int current_timeslot = initial_solution_lecture_timing.at(current_event);
			for (int l = 0; l < nb_locations; ++l)
			{
				// check if location available and feasible
				if (timeslot_location(current_timeslot, l) == -1
					&& get_sessionlocationpossible(current_event, l))
				{
					// set the event in that location
					solution_lecture_room.at(current_event) = l;

					// go to the next event
					generate_all_possible_solutions(current_event + 1);

					// when returning, reset the last assignment
					solution_lecture_room.at(current_event) = -1;
				}
			}
		}
	}


	// RECURSIVE FUNCTION TO GENERATE ALL SOLUTIONS FOR EACH TIMESLOT INDEPENDENTLY (for evacuations)
	void MIP_only_second_stage::generate_all_possible_solutions_independently(int timeslot, int current_event)
	{
		// solution has been generated entirely
		if (current_event >= nb_sessions)
		{
			fix_solution(solution_lecture_room);
			solve_problem();
			CPXdelrows(env, problem, CPXgetnumrows(env, problem) - nb_sessions, CPXgetnumrows(env, problem) - 1);

			++exhaustive_search_solutions_number;
			std::string text;
			text = "Solution," + std::to_string(exhaustive_search_solutions_number) + ",";
			for (int l = 0; l < nb_sessions; ++l)
				text += std::to_string(solution_lecture_room.at(l)) + "|";
			text += ",Objective_value," + std::to_string(objective_value);
			_logger << logger::log_type::INFORMATION << text;
		}
		else
		{
			// only if the event is planned in this timeslot, generate all possible rooms
			if (initial_solution_lecture_timing.at(current_event) == timeslot)
			{
				for (int l = 0; l < nb_locations; ++l)
				{
					// check if location available and feasible
					if (timeslot_location(timeslot, l) == -1
						&& get_sessionlocationpossible(current_event, l))
					{
						// set the event in that location
						solution_lecture_room.at(current_event) = l;

						// go to the next event
						generate_all_possible_solutions_independently(timeslot, current_event + 1);

						// when returning, reset the last assignment
						solution_lecture_room.at(current_event) = -1;
					}
				}
			}

		}
	}


	//////////////////////


	void MIP_only_second_stage::evaluate_solution(const std::vector<int> lecture_room)
	{
		initialize_cplex();
		build_problem();
		fix_solution(lecture_room);
		solve_problem();
		clear_cplex();

		write_output();
	}


	void MIP_only_second_stage::fix_solution(const std::vector<int> lecture_room)
	{
		int status = 0;
		double obj[1];						// Objective function
		double lb[1];						// Lower bound variables
		double ub[1];						// Upper bound variables
		double rhs[1];						// Right-hand side constraints
		char *colname[1];					// Variable names
		char *rowname[1];					// Constraint names
		char sense[1];						// Sign of constraint
		int matbeg[1];						// Begin position of the constraint
		std::unique_ptr<int[]> matind;		// Position of each element in constraint matrix
		std::unique_ptr<double[]> matval;	// Value of each element in constraint matrix
		char type[1];						// Type of variable (integer, binary, fractional)
		int f{ 0 };							// To calculate number of nonzero coefficients in each constraint

											// Create arrays
		matind = std::make_unique<int[]>(100000);
		matval = std::make_unique<double[]>(100000);

		// Fix x_ltc variables
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (initial_solution_lecture_timing.at(l) == t)
				{
					for (int c = 0; c < nb_locations; ++c)
					{
						if (lecture_room.at(l) == c)
						{
							sense[0] = 'E';
							rhs[0] = 1;

							std::string name = "Fix_w_" + std::to_string(l + 1) + "_" + std::to_string(c + 1);
							rowname[0] = const_cast<char*>(name.c_str());

							matbeg[0] = 0;
							f = 0;

							matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
							matval[f] = 1;
							++f;

							status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
							if (status != 0)
							{
								CPXgeterrorstring(env, status, error_text);
								throw std::runtime_error("Error in function MIP_monolithic::fix_solution(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
							}
						}
					}
				}
			}
		}


		// Write to file
		status = CPXwriteprob(env, problem, "MIP_model_second_stage.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}
	}


	/////////////////////

	void MIP_only_second_stage::write_output()
	{
		std::string text;


		// 1. instance name
		text = "Algorithm: "; text.append(algorithm_name);
		text += "\nProblem: " + instance_name_timetable + " + " + instance_name_building;

		// 2. settings
		text += "\nSettings:";
		text += "\n\tAlpha: " + std::to_string(_alpha);
		text += "\n\tObjective type: ";
		if (_objective_type == objective_type::maximum_over_timeslots)
			text += "maximum over timeslots";
		else
			text += "sum over timeslots";
		text += "\n\tTime limit (seconds): " + std::to_string(_time_limit);
		text += "\n\tOptimality tolerance: " + std::to_string(_optimality_tolerance);


		// 3. results
		text += "\nSolution status: " + solution_info;
		text += "\nElapsed time (seconds): " + std::to_string(get_computation_time());


		// 4. results
		text += "\nObjective value: " + std::to_string(objective_value);
		for (int t = 0; t < nb_timeslots; ++t)
		{
			if (_alpha > 0.01)
				text += "\n\tEvacuation time timeslot " + std::to_string(t + 1) + ": " + std::to_string(solution_evacuation_times.at(t));
			if (_alpha < 0.99)
				text += "\n\tTravel time timeslot " + std::to_string(t + 1) + ": " + std::to_string(solution_travel_times.at(t));
		}


		// 5. solution
		/*text += "\nSolution:";
		for (int t = 0; t < nb_timeslots; ++t)
		{
		text += "\nTimeslot: " + std::to_string(t + 1);
		for (int l = 0; l < nb_sessions; ++l)
		{
		if(initial_solution_lecture_timing.at(l) == t)
		text += "\n\tLecture " + std::to_string(l + 1) + " assigned to room " + location_names.at(solution_lecture_room.at(l));
		}
		}*/


		// Write single item to logger
		_logger << logger::log_type::INFORMATION << text;
	}





	int MIP_only_second_stage::timeslot_location(int timeslot, int location) const
	{
		for (int l = 0; l < nb_sessions; ++l) {
			if (initial_solution_lecture_timing.at(l) == timeslot && solution_lecture_room.at(l) == location) {
				return l;
			}
		}
		return -1; // no event planned in that timeslot & location
	}


	void MIP_only_second_stage::add_constraint_maximum_evacuation_time(double value)
	{
		int status = 0;
		double obj[1];						// Objective function
		double lb[1];						// Lower bound variables
		double ub[1];						// Upper bound variables
		double rhs[1];						// Right-hand side constraints
		char *colname[1];					// Variable names
		char *rowname[1];					// Constraint names
		char sense[1];						// Sign of constraint
		int matbeg[1];						// Begin position of the constraint
		std::unique_ptr<int[]> matind;		// Position of each element in constraint matrix
		std::unique_ptr<double[]> matval;	// Value of each element in constraint matrix
		char type[1];						// Type of variable (integer, binary, fractional)
		int f{ 0 };							// To calculate number of nonzero coefficients in each constraint

											// Create arrays
		matind = std::make_unique<int[]>(100000);
		matval = std::make_unique<double[]>(100000);


		sense[0] = 'L';
		rhs[0] = value;

		std::string name = "Constraint_on_maximum_evacuation_time";
		rowname[0] = const_cast<char*>(name.c_str());

		matbeg[0] = 0;
		f = 0;

		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			matind[f] = CPXgetnumcols(env, problem) - 1;
			matval[f] = 1;
			++f;
		}
		else // if (_objective_type == objective_type::sum_over_timeslots)
		{
			const int index_evacs = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths
				+ nb_timeslots + nb_timeslots * nb_sessions * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions * nb_paths;

			for (int t = 0; t < nb_timeslots; ++t)
			{
				matind[f] = index_evacs + t;
				matval[f] = 1;
				++f;
			}
		}

		status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::add_constraint_maximum_evacuation_time(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
		}


		// Write to file
		status = CPXwriteprob(env, problem, "MIP_model_second_stage.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::add_constraint_maximum_evacuation_time(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}
	}


}