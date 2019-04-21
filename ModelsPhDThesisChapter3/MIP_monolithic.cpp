#include "MIP_monolithic.h"
#include "timetable_data.h"
#include "building_data.h"
#include "initial_solution.h"
#include "logger.h"

#include <stdexcept>
#include <iostream>
#include <memory>


namespace alg
{
	void MIP_monolithic::run_algorithm(bool constraint_on_preferences, bool constraint_on_T, double value_pref, double value_T)
	{
		auto start_time = std::chrono::system_clock::now();

		initialize_cplex();
		build_problem();
		if (constraint_on_preferences)
			add_constraint_preferences(value_pref);
		if (constraint_on_T)
			add_constraint_TT(value_T);

		solve_problem();

		/*if (solution_exists)
		{
		if (_lambda < 0.01 || _lambda > 0.99)
		{
		update_objective_coefficients_analysis();
		fix_solution(solution_lecture_timeslot, solution_lecture_room);
		solve_problem();
		}
		}*/

		computation_time = std::chrono::system_clock::now() - start_time;

		write_output();

		clear_cplex();
	}


	void MIP_monolithic::initialize_cplex()
	{
		int status = 0;

		// Open the cplex environment
		env = CPXopenCPLEX(&status);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::initialize_cplex(). \nCouldn't open CPLEX. \nReason: " + std::string(error_text));
		}

		// Set the output to screen on/off
		status = CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_ON);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::initialize_cplex(). \nCouldn't set the output to screen off. \nReason: " + std::string(error_text));
		}

		// Set tolerance gap
		status = CPXsetdblparam(env, CPX_PARAM_EPGAP, _optimality_tolerance);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::initialize_cplex(). \nCouldn't set the tolerance gap. \nReason: " + std::string(error_text));
		}

		// Time limit for the MIP
		status = CPXsetdblparam(env, CPX_PARAM_TILIM, _time_limit);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::initialize_cplex(). \nCouldn't set the time limit. \nReason: " + std::string(error_text));
		}
	}


	void MIP_monolithic::build_problem()
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
		problem = CPXcreateprob(env, &status, "problem");
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't create the CPLEX problem. \nReason: " + std::string(error_text));
		}

		// Problem is minimization
		CPXchgobjsen(env, problem, CPX_MIN);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't set the problem type to minimization. \nReason: " + std::string(error_text));
		}

		// VARIABLES
		// Add the X_ltc variables
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int c = 0; c < nb_locations; ++c)
				{
					std::string name = "x_" + std::to_string(l + 1) + "_" + std::to_string(t + 1) + "_" + std::to_string(c + 1);
					colname[0] = const_cast<char*>(name.c_str());

					obj[0] = _lambda * (get_costsessiontimeslot(l, t) + 1000 * get_costsession_ts_educational(l, t));

					lb[0] = 0;
					ub[0] = 1;
					type[0] = 'B';

					status = CPXnewcols(env, problem, 1, obj, lb, ub, type, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
					}
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
						throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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
					throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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
						throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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
				obj[0] = (1 - _lambda)*(1 - _alpha);
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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
						throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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
					throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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
						throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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
				obj[0] = (1 - _lambda)*_alpha;
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(travel)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_travel_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = (1 - _lambda)*(1 - _alpha);
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(evac)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_evac_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = (1 - _lambda)*_alpha;
			lb[0] = 0;

			status = CPXnewcols(env, problem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}


		// CONSTRAINTS
		// TIMETABLE
		// Constraint set 1: Each Lecture should be scheduled and at an appropriate timeslot and location
		for (int l = 0; l < nb_sessions; ++l)
		{
			sense[0] = 'E';
			rhs[0] = 1;

			std::string name = "Lecture_" + std::to_string(l + 1) + "_scheduled";
			rowname[0] = const_cast<char*>(name.c_str());

			matbeg[0] = 0;
			f = 0;

			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int c = 0; c < nb_locations; ++c)
				{
					if (get_sessionlocationpossible(l, c))
					{
						matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
						matval[f] = 1;
						++f;
					}
				}
			}

			status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_sessionlocationpossible(l, c))
					{
						matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
						matval[f] = 1;
						++f;
					}
				}

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 3: No lecture conflicts
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int m = l + 1; m < nb_sessions; ++m)
				{
					if (get_sessionsessionconflict(l, m))
					{
						sense[0] = 'L';
						rhs[0] = 1;

						std::string name = "Session_" + std::to_string(l + 1) + "_and_" + std::to_string(m + 1) + "_no_conflict_time_" + std::to_string(t + 1);
						rowname[0] = const_cast<char*>(name.c_str());

						matbeg[0] = 0;
						f = 0;

						for (int c = 0; c < nb_locations; ++c)
						{
							if (get_sessionlocationpossible(l, c))
							{
								matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
								matval[f] = 1;
								++f;
							}

							if (get_sessionlocationpossible(m, c))
							{
								matind[f] = m * nb_timeslots*nb_locations + t * nb_locations + c;
								matval[f] = 1;
								++f;
							}
						}

						status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}

		// Teacher working time regulations
		// Constraint set 4: Not more than 4 lectures per day
		for (int d = 0; d < nb_teachers; ++d)
		{
			for (int day = 0; day < nb_days; ++day)
			{
				sense[0] = 'L';
				rhs[0] = 4;

				std::string name = "Teacher_" + std::to_string(d + 1) + "_not_more_than_8_hours_on_day_" + std::to_string(day + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_teachersession(d, l))
					{
						for (int t = day * nb_timeslots_per_day; t < (day + 1)*nb_timeslots_per_day; ++t)
						{
							for (int c = 0; c < nb_locations; ++c)
							{
								if (get_sessionlocationpossible(l, c))
								{
									matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
									matval[f] = 1;
									++f;
								}
							}
						}
					}
				}

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 5: Not more than 3 lectures consecutively
		for (int d = 0; d < nb_teachers; ++d)
		{
			for (int day = 0; day < nb_days; ++day)
			{
				sense[0] = 'L';
				rhs[0] = 3;

				std::string name = "Teacher_" + std::to_string(d + 1) + "_not_more_than_6_hours_consecutively_on_day_" + std::to_string(day + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_teachersession(d, l))
					{
						for (int t = day * nb_timeslots_per_day; t < day*nb_timeslots_per_day + nb_timeslots_per_day - 3; ++t)
						{
							for (int c = 0; c < nb_locations; ++c)
							{
								if (get_sessionlocationpossible(l, c))
								{
									// t
									matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
									matval[f] = 1;
									++f;

									// t + 1
									matind[f] = l * nb_timeslots*nb_locations + (t + 1) * nb_locations + c;
									matval[f] = 1;
									++f;

									// t + 2
									matind[f] = l * nb_timeslots*nb_locations + (t + 2) * nb_locations + c;
									matval[f] = 1;
									++f;

									// t + 3
									matind[f] = l * nb_timeslots*nb_locations + (t + 3) * nb_locations + c;
									matval[f] = 1;
									++f;
								}
							}
						}
					}
				}

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 6: Not first and last timeslot of same day
		for (int d = 0; d < nb_teachers; ++d)
		{
			for (int day = 0; day < nb_days; ++day)
			{
				sense[0] = 'L';
				rhs[0] = 1;

				std::string name = "Teacher_" + std::to_string(d + 1) + "_not_first_and_last_ts_on_day_" + std::to_string(day + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_teachersession(d, l))
					{
						for (int c = 0; c < nb_locations; ++c)
						{
							if (get_sessionlocationpossible(l, c))
							{
								// first timeslot of day
								matind[f] = l * nb_timeslots*nb_locations + day * nb_timeslots_per_day * nb_locations + c;
								matval[f] = 1;
								++f;

								// last timeslot of day
								matind[f] = l * nb_timeslots*nb_locations + ((day + 1)*nb_timeslots_per_day - 1) * nb_locations + c;
								matval[f] = 1;
								++f;
							}
						}
					}
				}

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 7: Not last timeslot of day and first of next day
		for (int d = 0; d < nb_teachers; ++d)
		{
			for (int day = 0; day + 1 < nb_days; ++day)
			{
				sense[0] = 'L';
				rhs[0] = 1;

				std::string name = "Teacher_" + std::to_string(d + 1) + "_not_last_and_first_on_next_day_" + std::to_string(day + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_teachersession(d, l))
					{
						for (int c = 0; c < nb_locations; ++c)
						{
							if (get_sessionlocationpossible(l, c))
							{
								// last timeslot of day
								matind[f] = l * nb_timeslots*nb_locations + ((day + 1)*nb_timeslots_per_day - 1) * nb_locations + c;
								matval[f] = 1;
								++f;

								// first timeslot of next day
								matind[f] = l * nb_timeslots*nb_locations + (day + 1)*nb_timeslots_per_day * nb_locations + c;
								matval[f] = 1;
								++f;


							}
						}
					}
				}

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Compactness constraints
		if (nb_timeslots_per_day == 5 || nb_timeslots_per_day == 6)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int day = 0; day < nb_days; ++day)
				{
					if (series_typeofeducation[s] == 0)
					{
						for (int t = 0; t < 3; ++t) // (1-3, 2-4,) and 3-5
						{
							sense[0] = 'L';
							rhs[0] = 1;

							std::string name = "Compactness_series_" + std::to_string(s + 1) + "_day_" + std::to_string(day + 1);
							rowname[0] = const_cast<char*>(name.c_str());

							matbeg[0] = 0;
							f = 0;

							for (int l = 0; l < nb_sessions; ++l)
							{
								if (get_seriessession(s, l))
								{
									for (int c = 0; c < nb_locations; ++c)
									{
										if (get_sessionlocationpossible(l, c))
										{
											// timeslot t
											matind[f] = l * nb_timeslots*nb_locations + (day*nb_timeslots_per_day + t) * nb_locations + c;
											matval[f] = 1;
											++f;

											// timeslot t + 2
											matind[f] = l * nb_timeslots*nb_locations + (day*nb_timeslots_per_day + t + 2) * nb_locations + c;
											matval[f] = 1;
											++f;

											// timeslot t + 1
											matind[f] = l * nb_timeslots*nb_locations + (day*nb_timeslots_per_day + t + 1) * nb_locations + c;
											matval[f] = -1;
											++f;
										}
									}
								}
							}

							status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
							if (status != 0)
							{
								CPXgeterrorstring(env, status, error_text);
								throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
							}
						}
					}
				}
			}
		}



		//////////////////////////////////////////////////////////////////////

		// FLOWS BETWEEN EVENTS IN CONSECUTIVE TIMESLOTS
		// Constraint set 8: U_tsp constraints 
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l))
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m))
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
														rhs[0] = get_roomroompath(c, d, p);

														std::string name = "TRAVELS_path_" + std::to_string(p + 1) + "_used_by_series_" + std::to_string(s + 1) + "_at_time_" + std::to_string(t + 1);
														rowname[0] = const_cast<char*>(name.c_str());

														matbeg[0] = 0;
														f = 0;

														// x_ltc
														matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
														matval[f] = get_roomroompath(c, d, p);
														++f;

														// x_l,t+1,d
														matind[f] = m * nb_timeslots*nb_locations + (t + 1) * nb_locations + d;
														matval[f] = get_roomroompath(c, d, p);
														++f;

														// U_tsp
														matind[f] = nb_sessions * nb_timeslots*nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
														matval[f] = -1;
														++f;


														status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_seriessession(s, l))
					{
						for (int c = 0; c < nb_locations; ++c)
						{
							if (get_sessionlocationpossible(l, c))
							{
								for (int p = 0; p < nb_paths; ++p)
								{
									if (get_roomroompath(c, nb_locations, p) > 0)
									{
										sense[0] = 'L';
										rhs[0] = 0;

										std::string name = "TRAVELS_path_" + std::to_string(p + 1) + "_used_by_series_" + std::to_string(s + 1) + "_at_time_" + std::to_string(t + 1);
										rowname[0] = const_cast<char*>(name.c_str());

										matbeg[0] = 0;
										f = 0;

										// x_ltc
										matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of x_m,t+1,d
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m))
											{
												for (int d = 0; d < nb_locations; ++d)
												{
													if (get_sessionlocationpossible(m, d))
													{
														// x_m,t+1,d
														matind[f] = m * nb_timeslots*nb_locations + (t + 1) * nb_locations + d;
														matval[f] = -get_roomroompath(c, nb_locations, p);
														++f;
													}
												}
											}
										}

										// U_tsp
										matind[f] = nb_sessions * nb_timeslots*nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
										matval[f] = -1;
										++f;

										status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_seriessession(s, l))
					{
						for (int c = 0; c < nb_locations; ++c)
						{
							if (get_sessionlocationpossible(l, c))
							{
								for (int p = 0; p < nb_paths; ++p)
								{
									if (get_roomroompath(c, nb_locations, p) > 0)
									{
										sense[0] = 'L';
										rhs[0] = 0;

										std::string name = "TRAVELS_path_" + std::to_string(p + 1) + "_used_by_series_" + std::to_string(s + 1) + "_at_time_" + std::to_string(t + 1);
										rowname[0] = const_cast<char*>(name.c_str());

										matbeg[0] = 0;
										f = 0;

										// x_l,t+1,c
										matind[f] = l * nb_timeslots*nb_locations + (t + 1) * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of x_mtd
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m))
											{
												for (int d = 0; d < nb_locations; ++d)
												{
													if (get_sessionlocationpossible(m, d))
													{
														matind[f] = m * nb_timeslots*nb_locations + t * nb_locations + d;
														matval[f] = -get_roomroompath(c, nb_locations, p);
														++f;
													}
												}
											}
										}

										// U_tsp
										matind[f] = nb_sessions * nb_timeslots*nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
										matval[f] = -1;
										++f;

										status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
						matind[f] = nb_sessions * nb_timeslots*nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
						matval[f] = -series_nb_people[s] * get_patharc(p, ij);
						++f;
					}
				}
				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
						matind[f] = nb_sessions * nb_timeslots*nb_locations + t * nb_series*nb_paths + s * nb_paths + p;
						matval[f] = -series_nb_people[s] * get_patharc(p, ij) * (double)arc_length[ij] / (walking_alpha*arc_area[ij]);
						if (arc_stairs[ij])
							matval[f] *= speed_correction_stairs;
						++f;
					}
				}

				// T(arc)_tij variable
				matind[f] = nb_sessions * nb_timeslots*nb_locations + nb_timeslots * nb_series*nb_paths + t * nb_arcs + ij;
				matval[f] = 1;
				++f;

				status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 11: Travel time for series s at time t
		for (int t = 0; t < nb_timeslots - 1; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l))
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m))
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

														// x_ltc
														matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
														matval[f] = Big_M;
														++f;

														// x_m,t+1,d
														matind[f] = m * nb_timeslots*nb_locations + (t + 1) * nb_locations + d;
														matval[f] = Big_M;
														++f;

														// sum of T_arc_t,ij
														for (int ij = 0; ij < nb_arcs; ++ij)
														{
															matind[f] = nb_sessions * nb_timeslots*nb_locations + nb_timeslots * nb_series*nb_paths + t * nb_arcs + ij;
															matval[f] = -get_patharc(p, ij);
															++f;
														}

														// T_tot_tsp
														matind[f] = nb_sessions * nb_timeslots*nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + t * nb_series*nb_paths + s * nb_paths + p;
														matval[f] = 1;
														++f;

														status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = 0; t < nb_timeslots - 1; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l))
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m))
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

														// x_ltc
														matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
														matval[f] = Big_M;
														++f;

														// x_m,t+1,d
														matind[f] = m * nb_timeslots*nb_locations + (t + 1) * nb_locations + d;
														matval[f] = Big_M;
														++f;

														// sum of T_arc_t,ij
														for (int ij = 0; ij < nb_arcs; ++ij)
														{
															matind[f] = nb_sessions * nb_timeslots*nb_locations + nb_timeslots * nb_series*nb_paths + t * nb_arcs + ij;
															matval[f] = get_patharc(p, ij);
															++f;
														}

														// T_tot_tsp
														matind[f] = nb_sessions * nb_timeslots*nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + t * nb_series*nb_paths + s * nb_paths + p;
														matval[f] = -1;
														++f;

														status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					matind[f] = nb_sessions * nb_timeslots*nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + t * nb_series*nb_paths + s * nb_paths + p;
					matval[f] = 1;
					++f;

					// T(travel)_t,max
					matind[f] = nb_sessions * nb_timeslots*nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series*nb_paths + t;
					matval[f] = -1;
					++f;

					status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}


		// EVACUATIONS
		const int nb_variables_first_part = nb_sessions * nb_timeslots * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series*nb_paths + nb_timeslots;

		// Constraint set 13: U_tlp constraints 
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int c = 0; c < nb_locations; ++c)
				{
					if (get_sessionlocationpossible(l, c))
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							if (get_roomroompath(c, nb_locations, p) > 0)
							{
								sense[0] = 'L';
								rhs[0] = 0;

								std::string name = "EVACS_path_" + std::to_string(p + 1) + "_used_by_lecture_" + std::to_string(l + 1) + "_at_time_" + std::to_string(t + 1);
								rowname[0] = const_cast<char*>(name.c_str());

								matbeg[0] = 0;
								f = 0;

								// x_ltc
								matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
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
									throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 16: Evacuation time for lecture l at time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int c = 0; c < nb_locations; ++c)
				{
					if (get_sessionlocationpossible(l, c))
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							if (get_roomroompath(c, nb_locations, p) > 0)
							{
								sense[0] = 'L';
								rhs[0] = Big_M;

								std::string name = "EVACS_travel_time_lecture_" + std::to_string(l + 1) + "_time_" + std::to_string(t + 1) + "_path_" + std::to_string(p + 1);
								rowname[0] = const_cast<char*>(name.c_str());

								matbeg[0] = 0;
								f = 0;

								// x_ltc
								matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
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
									throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
				for (int c = 0; c < nb_locations; ++c)
				{
					if (get_sessionlocationpossible(l, c))
					{
						for (int p = 0; p < nb_paths; ++p)
						{
							if (get_roomroompath(c, nb_locations, p) > 0)
							{
								sense[0] = 'L';
								rhs[0] = Big_M;

								std::string name = "EVACS_travel_time_lecture_" + std::to_string(l + 1) + "_time_" + std::to_string(t + 1) + "_path_" + std::to_string(p + 1);
								rowname[0] = const_cast<char*>(name.c_str());

								matbeg[0] = 0;
								f = 0;

								// x_ltc
								matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
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
									throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

					// T(evac)_t,max
					matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions*nb_paths + t;
					matval[f] = -1;
					++f;

					status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
						matind[f] = nb_sessions * nb_timeslots * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + t * nb_series*nb_paths + s * nb_paths + p;
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
							throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
							throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}


		// Write to file
		status = CPXwriteprob(env, problem, "MIP_model_monolithic.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}

		// Get info
		numcols_problem = CPXgetnumcols(env, problem);
		numrows_problem = CPXgetnumrows(env, problem);

		// Initialize vectors
		solution_lecture_timeslot.reserve(nb_sessions*nb_timeslots);
		solution_lecture_room.reserve(nb_sessions*nb_locations);
		for (int l = 0; l < nb_sessions; ++l)
		{
			solution_lecture_room.push_back(-1);
			solution_lecture_timeslot.push_back(-1);
		}

		// 2. travel times
		solution_travel_times.reserve(nb_timeslots);
		for (int t = 0; t < nb_timeslots; ++t)
		{
			solution_travel_times.push_back(0.0);
		}

		// 3. evacuation times
		solution_evacuation_times.reserve(nb_timeslots);
		for (int t = 0; t < nb_timeslots; ++t)
		{
			solution_evacuation_times.push_back(0.0);
		}

	}


	void MIP_monolithic::solve_problem()
	{
		int status = 0;
		int solstat;
		std::unique_ptr<double[]> solution_CPLEX = std::make_unique<double[]>(numcols_problem);


		// Optimize the problem
		std::cout << "\n\nCPLEX is solving the monolithic MIP for instance " << instance_name_timetable << " + " << instance_name_building << " ... ";
		status = CPXmipopt(env, problem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
		}

		// Get the solution
		status = CPXsolution(env, problem, &solstat, &objective_value, solution_CPLEX.get(), NULL, NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
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


		if (solution_exists)
		{
			std::cout << "\nObjective value: " << objective_value;

			// 1. solution: assignment of lectures to timeslots and rooms
			for (int l = 0; l < nb_sessions; ++l)
			{
				solution_lecture_room.at(l) = (-1);
				solution_lecture_timeslot.at(l) = (-1);

				for (int t = 0; t < nb_timeslots; ++t)
				{
					for (int r = 0; r < nb_locations; ++r)
					{
						if (solution_CPLEX[l*nb_timeslots*nb_locations + t * nb_locations + r] > 0.99)
						{
							solution_lecture_timeslot.at(l) = t;
							solution_lecture_room.at(l) = r;
							break;
						}
					}
				}
			}

			// 2. travel times
			{
				const int index_travels = nb_sessions * nb_timeslots * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					double time = solution_CPLEX[index_travels + t];
					solution_travel_times.at(t) = time;
				}
			}

			// 3. evacuation times
			{
				const int index_evacs = nb_sessions * nb_timeslots * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths
					+ nb_timeslots + nb_timeslots * nb_sessions * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions * nb_paths;
				for (int t = 0; t < nb_timeslots; ++t)
				{
					double time = solution_CPLEX[index_evacs + t];
					solution_evacuation_times.at(t) = time;
				}
			}

		}
	}


	void MIP_monolithic::clear_cplex()
	{
		int status = 0;

		// Clear vectors
		solution_lecture_timeslot.clear();
		solution_lecture_room.clear();
		solution_evacuation_times.clear();
		solution_travel_times.clear();

		// Free the problem
		status = CPXfreeprob(env, &problem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::clear_cplex(). \nCouldn't free the CPLEX problem. \nReason: " + std::string(error_text));
		}

		// Close the cplex environment
		status = CPXcloseCPLEX(&env);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::clear_cplex(). \nCouldn't close CPLEX environment. \nReason: " + std::string(error_text));
		}
	}


	void MIP_monolithic::write_output()
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
		text += "\n\tOptimality tolerance: " + std::to_string(_optimality_tolerance);


		// 3. results
		text += "\nSolution status: " + solution_info;
		text += "\nElapsed time (seconds): " + std::to_string(get_computation_time());


		// 4. results
		text += "\nObjective value: " + std::to_string(objective_value);
		{
			int pref = 0;
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int t = 0; t < nb_timeslots; ++t)
				{
					if (solution_lecture_timeslot.at(l) == t)
					{
						pref += get_costsessiontimeslot(l, t) + 1000 * get_costsession_ts_educational(l, t);
					}
				}
			}
			text += "\n\tPreferences: " + std::to_string(pref);
		}
		for (int t = 0; t < nb_timeslots; ++t)
		{
			if (_alpha > 0.01)
				text += "\n\tEvacuation time timeslot " + std::to_string(t + 1) + ": " + std::to_string(solution_evacuation_times.at(t));
			if (_alpha < 0.99)
				text += "\n\tTravel time timeslot " + std::to_string(t + 1) + ": " + std::to_string(solution_travel_times.at(t));
		}


		// 5. solution
		text += "\nSolution:";
		for (int t = 0; t < nb_timeslots; ++t)
		{
			text += "\nTimeslot: " + std::to_string(t + 1);
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (solution_lecture_timeslot.at(l) == t)
					text += "\n\tLecture " + session_names.at(l) + " assigned to room " + location_names.at(solution_lecture_room.at(l));
			}
		}


		// Write single item to logger
		_logger << logger::log_type::INFORMATION << text;
	}



	void MIP_monolithic::evaluate_solution(const std::vector<int> lecture_timeslot, const std::vector<int> lecture_room)
	{
		initialize_cplex();
		build_problem();
		fix_solution(lecture_timeslot, lecture_room);
		solve_problem();
		clear_cplex();

		write_output();
	}


	void MIP_monolithic::fix_solution(const std::vector<int> lecture_timeslot, const std::vector<int> lecture_room)
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
		matind = std::make_unique<int[]>(1000000);
		matval = std::make_unique<double[]>(1000000);

		// Fix x_ltc variables
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int c = 0; c < nb_locations; ++c)
				{
					sense[0] = 'E';

					if (lecture_timeslot.at(l) == t && lecture_room.at(l) == c)
						rhs[0] = 1;
					else
						rhs[0] = 0;

					std::string name = "Fix_x_" + std::to_string(l + 1) + "_" + std::to_string(t + 1) + "_" + std::to_string(c + 1);
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


		// Write to file
		status = CPXwriteprob(env, problem, "MIP_model_monolithic.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::build_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}
	}


	void MIP_monolithic::update_objective_coefficients_analysis()
	{
		_lambda = 0.5;
		_alpha = 0.5;

		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			int nonzeros = nb_sessions * nb_timeslots * nb_locations + 2;
			std::unique_ptr<int[]> indices = std::make_unique<int[]>(nonzeros);
			std::unique_ptr<double[]> values = std::make_unique<double[]>(nonzeros);

			// x_ltc
			int index = -1;
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int t = 0; t < nb_timeslots; ++t)
				{
					for (int c = 0; c < nb_locations; ++c)
					{
						++index;
						indices[index] = index;
						values[index] = _lambda * (get_costsessiontimeslot(l, t) + 1000 * get_costsession_ts_educational(l, t)); // lambda == 1
					}
				}
			}

			// travels
			indices[index] = CPXgetnumcols(env, problem) - 2;
			values[index] = (1 - _lambda) * (1 - _alpha);
			++index;

			// evac
			indices[index] = CPXgetnumcols(env, problem) - 2;
			values[index] = (1 - _lambda) * _alpha;
			++index;

			int status = CPXchgobj(env, problem, nonzeros, indices.get(), values.get());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_monolithic::update_objective_coefficients_analysis(). \nCouldn't change objective function coefficients.\nReason: " + std::string(error_text));
			}
		}
		else // if (_objective_type == objective_type::sum_over_timeslots)
		{
			int nonzeros = nb_sessions * nb_timeslots * nb_locations + 2 * nb_timeslots;
			std::unique_ptr<int[]> indices = std::make_unique<int[]>(nonzeros);
			std::unique_ptr<double[]> values = std::make_unique<double[]>(nonzeros);

			// x_ltc
			int index = -1;
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int t = 0; t < nb_timeslots; ++t)
				{
					for (int c = 0; c < nb_locations; ++c)
					{
						++index;
						indices[index] = index;
						values[index] = _lambda * (get_costsessiontimeslot(l, t) + 1000 * get_costsession_ts_educational(l, t)); // lambda == 1
					}
				}
			}

			// travels
			const int index_travels = nb_sessions * nb_timeslots * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths;
			for (int t = 0; t < nb_timeslots; ++t)
			{
				indices[index] = index_travels + t;
				values[index] = (1 - _lambda) * (1 - _alpha);
				++index;
			}

			// evac
			const int index_evacs = nb_sessions * nb_timeslots * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths
				+ nb_timeslots + nb_timeslots * nb_sessions * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions * nb_paths;
			for (int t = 0; t < nb_timeslots; ++t)
			{
				indices[index] = index_evacs + t;
				values[index] = (1 - _lambda) * _alpha;
				++index;
			}

			int status = CPXchgobj(env, problem, nonzeros, indices.get(), values.get());
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_monolithic::update_objective_coefficients_analysis(). \nCouldn't change objective function coefficients.\nReason: " + std::string(error_text));
			}
		}
	}




	void MIP_monolithic::add_constraint_preferences(double value)
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

		std::string name = "Constraint_on_maximum_preference_score";
		rowname[0] = const_cast<char*>(name.c_str());

		matbeg[0] = 0;
		f = 0;

		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				for (int c = 0; c < nb_locations; ++c)
				{
					if (get_sessionlocationpossible(l, c))
					{
						matind[f] = l * nb_timeslots*nb_locations + t * nb_locations + c;
						matval[f] = (get_costsessiontimeslot(l, t) + 1000 * get_costsession_ts_educational(l, t));
						++f;
					}
				}
			}
		}

		status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::add_constraint_preferences(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
		}


		// Write to file
		status = CPXwriteprob(env, problem, "MIP_model_monolithic.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::add_constraint_preferences(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}
	}


	void MIP_monolithic::add_constraint_TT(double value)
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

		std::string name = "Constraint_on_travel_or_evacuation_times";
		rowname[0] = const_cast<char*>(name.c_str());

		matbeg[0] = 0;
		f = 0;

		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			// 2. travel times
			{
				// Theta (Tmax) variable
				matind[f] = CPXgetnumcols(env, problem) - 2;
				matval[f] = 1;
				++f;
			}

			// 3. evacuation times
			{
				matind[f] = CPXgetnumcols(env, problem) - 1;
				matval[f] = 1;
				++f;
			}
		}
		else // _objective_type == objective_type::sum_over_timeslots)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				// T(travel)_t,max
				matind[f] = nb_sessions * nb_timeslots*nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series*nb_paths + t;
				matval[f] = 1;
				++f;

				// T(evac)_t,max
				const int nb_variables_first_part = nb_sessions * nb_timeslots * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series*nb_paths + nb_timeslots;
				matind[f] = nb_variables_first_part + nb_timeslots * nb_sessions*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions*nb_paths + t;
				matval[f] = 1;
				++f;
			}
		}

		status = CPXaddrows(env, problem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::add_constraint_TT(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
		}


		// Write to file
		status = CPXwriteprob(env, problem, "MIP_model_monolithic.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_monolithic::add_constraint_TT(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}
	}
}