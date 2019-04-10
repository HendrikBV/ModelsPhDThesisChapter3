#include "MIP_two_stage.h"
#include "timetable_data.h"
#include "building_data.h"
#include "initial_solution.h"
#include "logger.h"

#include <stdexcept>
#include <iostream>
#include <memory>



namespace alg
{
	void MIP_two_stage::run_algorithm_days_decomposition()
	{
		auto start_time = std::chrono::system_clock::now();

		initialize_cplex();

		build_masterproblem();
		solve_masterproblem();
		clear_masterproblem();

		if (!solution_firststage_exists)
			throw std::runtime_error("Couldn't solve second stage, because no solution for first stage exists. Reason: " + solution_info_firststage);

		computation_time_firststage = std::chrono::system_clock::now() - start_time;
		start_time = std::chrono::system_clock::now();

		objective_value_secondstage = 0;
		double TT = 0, ET = 0;
		for (int day = 0; day < nb_days; ++day)
		{
			for (int afternoon = 0; afternoon < 2; ++afternoon)
			{
				build_subproblem(day, afternoon);
				solve_subproblem(day, afternoon);
			}
		}
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			for (auto&& tt : solution_travel_times)
				if (tt > TT)
					TT = tt;
			for (auto&& et : solution_evacuation_times)
				if (et > ET)
					ET = et;
		}
		else // _objective_type == objective_type::sum_over_timeslots
		{
			for (auto&& tt : solution_travel_times)
				TT += tt;
			for (auto&& et : solution_evacuation_times)
				ET += et;
		}
		objective_value_secondstage = _alpha * ET + (1 - _alpha) * TT;

		clear_subproblem();
		clear_cplex();

		computation_time_secondstage = std::chrono::system_clock::now() - start_time;

		write_output();
	}


	void MIP_two_stage::run_algorithm_timeslot_decomposition()
	{
		auto start_time = std::chrono::system_clock::now();

		initialize_cplex();

		build_masterproblem();
		solve_masterproblem();
		clear_masterproblem();

		if (!solution_firststage_exists)
			throw std::runtime_error("Couldn't solve second stage, because no solution for first stage exists. Reason: " + solution_info_firststage);

		computation_time_firststage = std::chrono::system_clock::now() - start_time;
		start_time = std::chrono::system_clock::now();

		objective_value_secondstage = 0;
		double TT = 0, ET = 0;
		for (int t = 0; t < nb_timeslots; ++t)
		{
			build_subproblem(t);
			solve_subproblem(t);
		}
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			for (auto&& tt : solution_travel_times)
				if (tt > TT)
					TT = tt;
			for (auto&& et : solution_evacuation_times)
				if (et > ET)
					ET = et;
		}
		else // _objective_type == objective_type::sum_over_timeslots
		{
			for (auto&& tt : solution_travel_times)
				TT += tt;
			for (auto&& et : solution_evacuation_times)
				ET += et;
		}
		objective_value_secondstage = _alpha * ET + (1 - _alpha) * TT;

		clear_subproblem();
		clear_cplex();

		computation_time_secondstage = std::chrono::system_clock::now() - start_time;

		write_output();
	}



	void MIP_two_stage::run_algorithm(bool constraint_on_evacuation_time, double value)
	{
		auto start_time = std::chrono::system_clock::now();

		initialize_cplex();

		build_masterproblem();
		solve_masterproblem();
		clear_masterproblem();

		if (!solution_firststage_exists)
			throw std::runtime_error("Couldn't solve second stage, because no solution for first stage exists. Reason: " + solution_info_firststage);

		computation_time_firststage = std::chrono::system_clock::now() - start_time;
		start_time = std::chrono::system_clock::now();

		build_subproblem();
		if (constraint_on_evacuation_time)
			add_constraint_maximum_evacuation_time(value);

		solve_subproblem();
		clear_subproblem();

		clear_cplex();

		computation_time_secondstage = std::chrono::system_clock::now() - start_time;

		write_output();
	}


	void MIP_two_stage::run_only_first_stage()
	{
		auto start_time = std::chrono::system_clock::now();

		initialize_cplex();

		build_masterproblem();
		solve_masterproblem();
		clear_masterproblem();
		clear_cplex();

		computation_time_firststage = std::chrono::system_clock::now() - start_time;

		write_output();
	}




	void MIP_two_stage::initialize_cplex()
	{
		int status = 0;

		// Open the cplex environment
		env = CPXopenCPLEX(&status);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::initialize_cplex(). \nCouldn't open CPLEX. \nReason: " + std::string(error_text));
		}

		// Set the output to screen on/off
		status = CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::initialize_cplex(). \nCouldn't set the output to screen off. \nReason: " + std::string(error_text));
		}

		// Set tolerance gap
		status = CPXsetdblparam(env, CPX_PARAM_EPGAP, _optimality_tolerance);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::initialize_cplex(). \nCouldn't set the tolerance gap. \nReason: " + std::string(error_text));
		}

		// Time limit for the MIP
		status = CPXsetdblparam(env, CPX_PARAM_TILIM, _time_limit);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::initialize_cplex(). \nCouldn't set the time limit. \nReason: " + std::string(error_text));
		}
	}




	void MIP_two_stage::build_masterproblem()
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
		matind = std::make_unique<int[]>(1000000);
		matval = std::make_unique<double[]>(1000000);

		// Create the masterproblem
		masterproblem = CPXcreateprob(env, &status, "masterproblem");
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't create the CPLEX masterproblem. \nReason: " + std::string(error_text));
		}

		// masterproblem is minimization
		CPXchgobjsen(env, masterproblem, CPX_MIN);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't set the masterproblem type to minimization. \nReason: " + std::string(error_text));
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

					obj[0] = (get_costsessiontimeslot(l, t) + 1000 * get_costsession_ts_educational(l, t));

					lb[0] = 0;
					ub[0] = 1;
					type[0] = 'B';

					status = CPXnewcols(env, masterproblem, 1, obj, lb, ub, type, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
					}
				}
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

			status = CPXaddrows(env, masterproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXaddrows(env, masterproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

						status = CPXaddrows(env, masterproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
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

				status = CPXaddrows(env, masterproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
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

				status = CPXaddrows(env, masterproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXaddrows(env, masterproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
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

				status = CPXaddrows(env, masterproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
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
						for (int t = 2; t < 3; ++t) // (1-3, 2-4,) and 3-5
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

							status = CPXaddrows(env, masterproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
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



		// Write to file
		status = CPXwriteprob(env, masterproblem, "MIP_model_first_stage.lp", NULL);
		/*if (status != 0)
		{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nFailed to write the masterproblem to a file. \nReason: " + std::string(error_text));
		}*/

		// Get model info
		nb_variables_firststage = CPXgetnumcols(env, masterproblem);
		nb_constraints_firststage = CPXgetnumrows(env, masterproblem);
	}


	void MIP_two_stage::solve_masterproblem()
	{
		int status = 0;
		int solstat;
		std::unique_ptr<double[]> solution_CPLEX = std::make_unique<double[]>(CPXgetnumcols(env, masterproblem));


		// Optimize the masterproblem
		std::cout << "\n\nCPLEX is solving the first stage MIP for instance " << instance_name_timetable << " + " << instance_name_building << " ... ";
		status = CPXmipopt(env, masterproblem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
		}

		// Get the solution
		status = CPXsolution(env, masterproblem, &solstat, &objective_value_firststage, solution_CPLEX.get(), NULL, NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
		}


		// Check the solution status
		if (solstat == CPXMIP_OPTIMAL) {
			solution_firststage_exists = true;
			solution_info_firststage = "Optimal solution found";
		}
		else if (solstat == CPXMIP_OPTIMAL_TOL) {
			solution_firststage_exists = true;
			solution_info_firststage = "Solution found within tolerance limit";
		}
		else if (solstat == CPXMIP_TIME_LIM_FEAS) {
			solution_firststage_exists = true;
			solution_info_firststage = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_FEAS) {
			solution_firststage_exists = true;
			solution_info_firststage = "Tree memory limit exceeded";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_firststage_exists = false;
			solution_info_firststage = "masterproblem is infeasible";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_firststage_exists = false;
			solution_info_firststage = "masterproblem is infeasible";
		}
		else if (solstat == CPXMIP_UNBOUNDED) {
			solution_firststage_exists = false;
			solution_info_firststage = "masterproblem is unbounded";
		}
		else if (solstat == CPXMIP_INForUNBD) {
			solution_firststage_exists = false;
			solution_info_firststage = "masterproblem is infeasible or unbounded";
		}
		else if (solstat == CPXMIP_TIME_LIM_INFEAS) {
			solution_firststage_exists = false;
			solution_info_firststage = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_INFEAS) {
			solution_firststage_exists = false;
			solution_info_firststage = "Tree memory limit exceeded";
		}
		else {
			solution_firststage_exists = false;
			solution_info_firststage = "Other reason for failure";
		}

		std::cout << "\nCPLEX has finished: " << solution_info_firststage << "\n";


		if (solution_firststage_exists)
		{
			// 1. solution: assignment of lectures to timeslots and rooms
			solution_lecture_timeslot.reserve(nb_sessions*nb_timeslots);
			solution_lecture_room.reserve(nb_sessions*nb_locations);
			for (int l = 0; l < nb_sessions; ++l)
			{
				solution_lecture_room.push_back(-1);
				solution_lecture_timeslot.push_back(-1);

				for (int t = 0; t < nb_timeslots; ++t)
				{
					for (int r = 0; r < nb_locations; ++r)
					{
						if (solution_CPLEX[l*nb_timeslots*nb_locations + t * nb_locations + r] > 0.99)
						{
							solution_lecture_timeslot.back() = t;
							solution_lecture_room.back() = r;
							break;
						}
					}
				}
			}
		}
	}


	void MIP_two_stage::clear_masterproblem()
	{
		int status = 0;

		// Free the masterproblem
		status = CPXfreeprob(env, &masterproblem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::clear_masterproblem(). \nCouldn't free the masterproblem. \nReason: " + std::string(error_text));
		}
	}



	void MIP_two_stage::build_subproblem()
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

		// Create the subproblem
		subproblem = CPXcreateprob(env, &status, "problem_second_stage");
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't create the CPLEX subproblem. \nReason: " + std::string(error_text));
		}

		// Subproblem is minimization
		CPXchgobjsen(env, subproblem, CPX_MIN);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't set the subproblem type to minimization. \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, ub, type, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(travel)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_travel_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = (1 - _alpha);
			lb[0] = 0;

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(evac)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_evac_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = _alpha;
			lb[0] = 0;

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

			status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_sessionlocationpossible(l, c) && solution_lecture_timeslot[l] == t)
					{
						matind[f] = l * nb_locations + c;
						matval[f] = 1;
						++f;
					}
				}

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t) // only lectures planned in this timeslot
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1) // lectures planned in the next timeslot
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


														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
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

										// w_lc
										matind[f] = l * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of w_md
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

										status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t + 1)
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

										// w_lc
										matind[f] = l * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of w_md
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t)
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

										status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
				if (solution_lecture_timeslot[l] == t) // only lectures planned in this timeslot
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

									// w_lc
									matind[f] = l * nb_locations + c;
									matval[f] = get_roomroompath(c, nb_locations, p);
									++f;

									// U_tlp
									matind[f] = nb_variables_first_part + t * nb_sessions*nb_paths + l * nb_paths + p;
									matval[f] = -1;
									++f;


									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 16: Evacuation time for lecture l at time t
		for (int t = 0; t < nb_timeslots; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (solution_lecture_timeslot[l] == t)
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

									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
				if (solution_lecture_timeslot[l] == t)
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

									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
						matind[f] = CPXgetnumcols(env, subproblem) - 2;
						matval[f] = 1;
						++f;

						status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
						matind[f] = CPXgetnumcols(env, subproblem) - 1;
						matval[f] = 1;
						++f;

						status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}


		// Write to file
		status = CPXwriteprob(env, subproblem, "MIP_model_second_stage.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nFailed to write the subproblem to a file. \nReason: " + std::string(error_text));
		}

		// Get model info
		nb_variables_firststage = CPXgetnumcols(env, masterproblem);
		nb_constraints_firststage = CPXgetnumrows(env, masterproblem);
	}


	void MIP_two_stage::solve_subproblem()
	{
		int status = 0;
		int solstat;
		std::unique_ptr<double[]> solution_CPLEX = std::make_unique<double[]>(CPXgetnumcols(env, subproblem));


		// Optimize the problem
		std::cout << "\n\nCPLEX is solving the second stage MIP for instance " << instance_name_timetable << " + " << instance_name_building << " ... ";
		status = CPXmipopt(env, subproblem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
		}

		// Get the solution
		status = CPXsolution(env, subproblem, &solstat, &objective_value_secondstage, solution_CPLEX.get(), NULL, NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
		}


		// Check the solution status
		if (solstat == CPXMIP_OPTIMAL) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Optimal solution found";
		}
		else if (solstat == CPXMIP_OPTIMAL_TOL) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Solution found within tolerance limit";
		}
		else if (solstat == CPXMIP_TIME_LIM_FEAS) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_FEAS) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Tree memory limit exceeded";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible";
		}
		else if (solstat == CPXMIP_UNBOUNDED) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is unbounded";
		}
		else if (solstat == CPXMIP_INForUNBD) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible or unbounded";
		}
		else if (solstat == CPXMIP_TIME_LIM_INFEAS) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_INFEAS) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Tree memory limit exceeded";
		}
		else {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Other reason for failure";
		}

		std::cout << "\nCPLEX has finished: " << solution_info_secondstage << "\n";


		if (solution_secondstage_exists)
		{
			// 1. solution: assignment of lectures to rooms
			solution_lecture_room.reserve(nb_sessions*nb_locations);
			for (int l = 0; l < nb_sessions; ++l)
			{
				solution_lecture_room.push_back(-1);
				for (int r = 0; r < nb_locations; ++r)
				{
					if (solution_CPLEX[l*nb_locations + r] > 0.99)
					{
						solution_lecture_room.back() = r;
						break;
					}
				}
			}

			// 2. travel times
			{
				const int index_travels = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths;
				solution_travel_times.reserve(nb_timeslots);
				for (int t = 0; t < nb_timeslots; ++t)
				{
					double time = solution_CPLEX[index_travels + t];
					solution_travel_times.push_back(time);
				}
			}

			// 3. evacuation times
			{
				const int index_evacs = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths
					+ nb_timeslots + nb_timeslots * nb_sessions * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions * nb_paths;
				solution_evacuation_times.reserve(nb_timeslots);
				for (int t = 0; t < nb_timeslots; ++t)
				{
					double time = solution_CPLEX[index_evacs + t];
					solution_evacuation_times.push_back(time);
				}
			}
		}
	}


	void MIP_two_stage::build_subproblem(int day, int afternoon)
	{
		constexpr int Big_M = 10000;
		int status = 0;
		double obj[1];					// Objective function
		double lb[1];					// Lower bound variables
		double ub[1];					// Upper bound variables
		double rhs[1];					// Right-hand side constraints
		char *colname[1];				// Variable names
		char *rowname[1];				// Constraint names
		char sense[1];					// Sign of constraint
		int matbeg[1];					// Begin position of the constraint
		std::unique_ptr<int[]> matind;		// Position of each element in constraint matrix
		std::unique_ptr<double[]> matval;	// Value of each element in constraint matrix
		char type[1];					// Type of variable (integer, binary, fractional)
		int f{ 0 };						// To calculate number of nonzero coefficients in each constraint

										// create arrays
		matind = std::make_unique<int[]>(10000000);
		matval = std::make_unique<double[]>(10000000);


		// for which timeslots does this decomposed problem need to be constructed
		int first_timeslot, last_timeslot;

		if (nb_timeslots_per_day == 5)
		{
			first_timeslot = day * nb_timeslots_per_day + afternoon * 2;
			last_timeslot = day * nb_timeslots_per_day + 1 + afternoon * 3;
		}
		else if (nb_timeslots_per_day == 6)
		{
			first_timeslot = day * nb_timeslots_per_day + afternoon * 2;
			last_timeslot = day * nb_timeslots_per_day + 1 + afternoon * 4;
		}
		else if (nb_timeslots_per_day == 9)
		{
			first_timeslot = day * nb_timeslots_per_day + afternoon * 3;
			last_timeslot = day * nb_timeslots_per_day + 2 + afternoon * 6;
		}
		else
		{
			first_timeslot = 0;
			last_timeslot = nb_timeslots - 1;
		}


		// First delete previous problem and then create the new problem
		status = CPXfreeprob(env, &subproblem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_subproblem(). \nCouldn't free the previous subproblem. \nReason: " + std::string(error_text));
		}

		subproblem = CPXcreateprob(env, &status, "subproblem");
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_subproblem(). \nCouldn't create the CPLEX problem. \nReason: " + std::string(error_text));
		}

		// Problem is minimization
		CPXchgobjsen(env, subproblem, CPX_MIN);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_subproblem(). \nCouldn't set the problem type to minimization. \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, ub, type, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(travel)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_travel_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = (1 - _alpha);
			lb[0] = 0;

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(evac)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_evac_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = _alpha;
			lb[0] = 0;

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}


		// CONSTRAINTS
		// TIMETABLE
		// Constraint set 1: Each Lecture should be scheduled and at an appropriate location
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int t = first_timeslot; t <= last_timeslot; ++t) // only for lectures planned in this decomposed part
			{
				if (solution_lecture_timeslot[l] == t)
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

					status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}

		// Constraint set 1bis: Lectures not included in this decomposed part set to zero
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t < first_timeslot || t > last_timeslot)
				{
					if (solution_lecture_timeslot[l] == t)
					{
						for (int c = 0; c < nb_locations; ++c)
						{
							sense[0] = 'E';
							rhs[0] = 0;

							std::string name = "Lecture_" + std::to_string(l + 1) + "_not_scheduled_in_room" + std::to_string(c + 1);
							rowname[0] = const_cast<char*>(name.c_str());

							matbeg[0] = 0;
							f = 0;

							matind[f] = l * nb_locations + c;
							matval[f] = 1;
							++f;

							status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
							if (status != 0)
							{
								CPXgeterrorstring(env, status, error_text);
								throw std::runtime_error("Error in function Two_Stage_Model::build_subproblem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
							}
						}
					}
				}
			}
		}


		// Constraint set 2: No room overlaps
		for (int t = first_timeslot; t <= last_timeslot; ++t)
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
					if (get_sessionlocationpossible(l, c) && solution_lecture_timeslot[l] == t)
					{
						matind[f] = l * nb_locations + c;
						matval[f] = 1;
						++f;
					}
				}

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}


		// FLOWS BETWEEN EVENTS IN CONSECUTIVE TIMESLOTS
		// Constraint set 8: U_tsp constraints 
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t) // only lectures planned in this timeslot
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1) // lectures planned in the next timeslot
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


														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
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

										// w_lc
										matind[f] = l * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of w_md
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

										status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t + 1)
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

										// w_lc
										matind[f] = l * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of w_md
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t)
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

										status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
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
				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 10: Travel time through arc ij at time t
		for (int t = first_timeslot; t < last_timeslot; ++t)
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 11: Travel time for series s at time t
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
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

					status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}


		// EVACUATIONS
		const int nb_variables_first_part = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series*nb_paths + nb_timeslots;

		// Constraint set 13: U_tlp constraints 
		for (int t = first_timeslot; t <= last_timeslot; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (solution_lecture_timeslot[l] == t) // only lectures planned in this timeslot
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

									// w_lc
									matind[f] = l * nb_locations + c;
									matval[f] = get_roomroompath(c, nb_locations, p);
									++f;

									// U_tlp
									matind[f] = nb_variables_first_part + t * nb_sessions*nb_paths + l * nb_paths + p;
									matval[f] = -1;
									++f;


									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 14: maximum crowd flow per arc
		for (int t = first_timeslot; t <= last_timeslot; ++t)
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 15: Evacuation time through arc ij at time t
		for (int t = first_timeslot; t <= last_timeslot; ++t)
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 16: Evacuation time for lecture l at time t
		for (int t = first_timeslot; t <= last_timeslot; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (solution_lecture_timeslot[l] == t)
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

									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 16bis: Evacuation time for lecture l at time t
		for (int t = first_timeslot; t <= last_timeslot; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (solution_lecture_timeslot[l] == t)
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

									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 17: Maximum evacuation time timeslot t
		for (int t = first_timeslot; t <= last_timeslot; ++t)
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

					status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}


		// MAXIMUM
		// Constraint set 18: Maximal travel time
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			for (int t = first_timeslot; t < last_timeslot; ++t)
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
						matind[f] = CPXgetnumcols(env, subproblem) - 2;
						matval[f] = 1;
						++f;

						status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}

		// Constraint set 19: Maximal evacuation time
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			for (int t = first_timeslot; t <= last_timeslot; ++t)
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
						matind[f] = CPXgetnumcols(env, subproblem) - 1;
						matval[f] = 1;
						++f;

						status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}


		// Write to file
		status = CPXwriteprob(env, subproblem, "MIP_model_second_stage.lp", NULL);
		/*if (status != 0)
		{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nFailed to write the subproblem to a file. \nReason: " + std::string(error_text));
		}*/


		// Get model info
		nb_variables_firststage = CPXgetnumcols(env, masterproblem);
		nb_constraints_firststage = CPXgetnumrows(env, masterproblem);
	}


	void MIP_two_stage::solve_subproblem(int day, int afternoon)
	{
		int status = 0;
		int solstat;
		std::unique_ptr<double[]> solution_CPLEX = std::make_unique<double[]>(CPXgetnumcols(env, subproblem));


		// Optimize the problem
		if (afternoon == 0)
			std::cout << "\n\nCPLEX is solving the second stage MIP, for day " << day + 1 << " morning, for instance " << instance_name_timetable << " + " << instance_name_building << " ... ";
		else
			std::cout << "\n\nCPLEX is solving the second stage MIP, for day " << day + 1 << " afternoon, for instance " << instance_name_timetable << " + " << instance_name_building << " ... ";
		status = CPXmipopt(env, subproblem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
		}

		// Get the solution
		status = CPXsolution(env, subproblem, &solstat, &objective_value_secondstage, solution_CPLEX.get(), NULL, NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
		}


		// Check the solution status
		if (solstat == CPXMIP_OPTIMAL) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Optimal solution found";
		}
		else if (solstat == CPXMIP_OPTIMAL_TOL) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Solution found within tolerance limit";
		}
		else if (solstat == CPXMIP_TIME_LIM_FEAS) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_FEAS) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Tree memory limit exceeded";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible";
		}
		else if (solstat == CPXMIP_UNBOUNDED) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is unbounded";
		}
		else if (solstat == CPXMIP_INForUNBD) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible or unbounded";
		}
		else if (solstat == CPXMIP_TIME_LIM_INFEAS) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_INFEAS) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Tree memory limit exceeded";
		}
		else {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Other reason for failure";
		}

		std::cout << "\nCPLEX has finished: " << solution_info_secondstage << "\n";


		if (solution_secondstage_exists)
		{
			// for which timeslots does this decomposed problem need to be constructed
			int first_timeslot, last_timeslot;

			if (nb_timeslots_per_day == 5)
			{
				first_timeslot = day * nb_timeslots_per_day + afternoon * 2;
				last_timeslot = day * nb_timeslots_per_day + 1 + afternoon * 3;
			}
			else if (nb_timeslots_per_day == 6)
			{
				first_timeslot = day * nb_timeslots_per_day + afternoon * 2;
				last_timeslot = day * nb_timeslots_per_day + 1 + afternoon * 4;
			}
			else if (nb_timeslots_per_day == 9)
			{
				first_timeslot = day * nb_timeslots_per_day + afternoon * 3;
				last_timeslot = day * nb_timeslots_per_day + 2 + afternoon * 6;
			}
			else
			{
				first_timeslot = 0;
				last_timeslot = nb_timeslots - 1;
			}

			// 1. travel times
			{
				const int index_travels = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths;
				solution_travel_times.reserve(last_timeslot - first_timeslot);
				for (int t = first_timeslot; t < last_timeslot; ++t)
				{
					double time = solution_CPLEX[index_travels + t];
					solution_travel_times.push_back(time);
				}
			}

			// 2. evacuation times
			{
				const int index_evacs = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths
					+ nb_timeslots + nb_timeslots * nb_sessions * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions * nb_paths;
				solution_evacuation_times.reserve(last_timeslot - first_timeslot + 1);
				for (int t = first_timeslot; t <= last_timeslot; ++t)
				{
					double time = solution_CPLEX[index_evacs + t];
					solution_evacuation_times.push_back(time);
				}
			}
		}
	}


	void MIP_two_stage::build_subproblem(int timeslot)
	{
		constexpr int Big_M = 10000;
		int status = 0;
		double obj[1];					// Objective function
		double lb[1];					// Lower bound variables
		double ub[1];					// Upper bound variables
		double rhs[1];					// Right-hand side constraints
		char *colname[1];				// Variable names
		char *rowname[1];				// Constraint names
		char sense[1];					// Sign of constraint
		int matbeg[1];					// Begin position of the constraint
		std::unique_ptr<int[]> matind;		// Position of each element in constraint matrix
		std::unique_ptr<double[]> matval;	// Value of each element in constraint matrix
		char type[1];					// Type of variable (integer, binary, fractional)
		int f{ 0 };						// To calculate number of nonzero coefficients in each constraint

										// create arrays
		matind = std::make_unique<int[]>(10000000);
		matval = std::make_unique<double[]>(10000000);


		// for which timeslots does this decomposed problem need to be constructed
		int first_timeslot = timeslot, last_timeslot = timeslot;


		// First delete previous problem and then create the new problem
		status = CPXfreeprob(env, &subproblem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_subproblem(). \nCouldn't free the previous subproblem. \nReason: " + std::string(error_text));
		}

		subproblem = CPXcreateprob(env, &status, "subproblem");
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_subproblem(). \nCouldn't create the CPLEX problem. \nReason: " + std::string(error_text));
		}

		// Problem is minimization
		CPXchgobjsen(env, subproblem, CPX_MIN);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_subproblem(). \nCouldn't set the problem type to minimization. \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, ub, type, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

				status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

					status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
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

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(travel)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_travel_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = (1 - _alpha);
			lb[0] = 0;

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}

		// Add the T(evac)_max_overall variable
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			std::string name = "T_evac_max_overall";
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = _alpha;
			lb[0] = 0;

			status = CPXnewcols(env, subproblem, 1, obj, lb, NULL, NULL, colname); // Generate columns (the variables) and subsequently add rows (constraints)
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}


		// CONSTRAINTS
		// TIMETABLE
		// Constraint set 1: Each Lecture should be scheduled and at an appropriate location
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int t = first_timeslot; t <= last_timeslot; ++t) // only for lectures planned in this decomposed part
			{
				if (solution_lecture_timeslot[l] == t)
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

					status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}

		// Constraint set 1bis: Lectures not included in this decomposed part set to zero
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int t = 0; t < nb_timeslots; ++t)
			{
				if (t < first_timeslot || t > last_timeslot)
				{
					if (solution_lecture_timeslot[l] == t)
					{
						for (int c = 0; c < nb_locations; ++c)
						{
							sense[0] = 'E';
							rhs[0] = 0;

							std::string name = "Lecture_" + std::to_string(l + 1) + "_not_scheduled_in_room" + std::to_string(c + 1);
							rowname[0] = const_cast<char*>(name.c_str());

							matbeg[0] = 0;
							f = 0;

							matind[f] = l * nb_locations + c;
							matval[f] = 1;
							++f;

							status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
							if (status != 0)
							{
								CPXgeterrorstring(env, status, error_text);
								throw std::runtime_error("Error in function Two_Stage_Model::build_subproblem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
							}
						}
					}
				}
			}
		}


		// Constraint set 2: No room overlaps
		for (int t = first_timeslot; t <= last_timeslot; ++t)
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
					if (get_sessionlocationpossible(l, c) && solution_lecture_timeslot[l] == t)
					{
						matind[f] = l * nb_locations + c;
						matval[f] = 1;
						++f;
					}
				}

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}


		// FLOWS BETWEEN EVENTS IN CONSECUTIVE TIMESLOTS
		// Constraint set 8: U_tsp constraints 
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t) // only lectures planned in this timeslot
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1) // lectures planned in the next timeslot
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


														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
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

										// w_lc
										matind[f] = l * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of w_md
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

										status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t + 1)
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

										// w_lc
										matind[f] = l * nb_locations + c;
										matval[f] = get_roomroompath(c, nb_locations, p);
										++f;

										// sum of w_md
										for (int m = 0; m < nb_sessions; ++m)
										{
											if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t)
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

										status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
										if (status != 0)
										{
											CPXgeterrorstring(env, status, error_text);
											throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
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
				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 10: Travel time through arc ij at time t
		for (int t = first_timeslot; t < last_timeslot; ++t)
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 11: Travel time for series s at time t
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
		{
			for (int s = 0; s < nb_series; ++s)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					if (get_seriessession(s, l) && solution_lecture_timeslot[l] == t)
					{
						for (int m = 0; m < nb_sessions; ++m)
						{
							if (get_seriessession(s, m) && solution_lecture_timeslot[m] == t + 1)
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

														status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
														if (status != 0)
														{
															CPXgeterrorstring(env, status, error_text);
															throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
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
		for (int t = first_timeslot; t < last_timeslot; ++t)
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

					status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}


		// EVACUATIONS
		const int nb_variables_first_part = nb_sessions * nb_locations + nb_timeslots * nb_series*nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series*nb_paths + nb_timeslots;

		// Constraint set 13: U_tlp constraints 
		for (int t = first_timeslot; t <= last_timeslot; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (solution_lecture_timeslot[l] == t) // only lectures planned in this timeslot
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

									// w_lc
									matind[f] = l * nb_locations + c;
									matval[f] = get_roomroompath(c, nb_locations, p);
									++f;

									// U_tlp
									matind[f] = nb_variables_first_part + t * nb_sessions*nb_paths + l * nb_paths + p;
									matval[f] = -1;
									++f;


									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 14: maximum crowd flow per arc
		for (int t = first_timeslot; t <= last_timeslot; ++t)
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 15: Evacuation time through arc ij at time t
		for (int t = first_timeslot; t <= last_timeslot; ++t)
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

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}

		// Constraint set 16: Evacuation time for lecture l at time t
		for (int t = first_timeslot; t <= last_timeslot; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (solution_lecture_timeslot[l] == t)
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

									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 16bis: Evacuation time for lecture l at time t
		for (int t = first_timeslot; t <= last_timeslot; ++t)
		{
			for (int l = 0; l < nb_sessions; ++l)
			{
				if (solution_lecture_timeslot[l] == t)
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

									status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
									if (status != 0)
									{
										CPXgeterrorstring(env, status, error_text);
										throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
									}
								}
							}
						}
					}
				}
			}
		}

		// Constraint set 17: Maximum evacuation time timeslot t
		for (int t = first_timeslot; t <= last_timeslot; ++t)
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

					status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
					if (status != 0)
					{
						CPXgeterrorstring(env, status, error_text);
						throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
					}
				}
			}
		}


		// MAXIMUM
		// Constraint set 18: Maximal travel time
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			for (int t = first_timeslot; t < last_timeslot; ++t)
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
						matind[f] = CPXgetnumcols(env, subproblem) - 2;
						matval[f] = 1;
						++f;

						status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}

		// Constraint set 19: Maximal evacuation time
		if (_objective_type == objective_type::maximum_over_timeslots)
		{
			for (int t = first_timeslot; t <= last_timeslot; ++t)
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
						matind[f] = CPXgetnumcols(env, subproblem) - 1;
						matval[f] = 1;
						++f;

						status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
						if (status != 0)
						{
							CPXgeterrorstring(env, status, error_text);
							throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
						}
					}
				}
			}
		}


		// Write to file
		status = CPXwriteprob(env, subproblem, "MIP_model_second_stage.lp", NULL);
		/*if (status != 0)
		{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nFailed to write the subproblem to a file. \nReason: " + std::string(error_text));
		}*/


		// Get model info
		nb_variables_firststage = CPXgetnumcols(env, masterproblem);
		nb_constraints_firststage = CPXgetnumrows(env, masterproblem);
	}


	void MIP_two_stage::solve_subproblem(int timeslot)
	{
		int status = 0;
		int solstat;
		std::unique_ptr<double[]> solution_CPLEX = std::make_unique<double[]>(CPXgetnumcols(env, subproblem));


		// Optimize the problem
		std::cout << "\n\nCPLEX is solving the second stage MIP, for timeslot " << timeslot + 1 << ", for instance " << instance_name_timetable << " + " << instance_name_building << " ... ";
		status = CPXmipopt(env, subproblem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::solve_problem(). \nCPXmipopt failed. \nReason: " + std::string(error_text));
		}

		// Get the solution
		status = CPXsolution(env, subproblem, &solstat, &objective_value_secondstage, solution_CPLEX.get(), NULL, NULL, NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_only_second_stage::solve_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
		}


		// Check the solution status
		if (solstat == CPXMIP_OPTIMAL) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Optimal solution found";
		}
		else if (solstat == CPXMIP_OPTIMAL_TOL) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Solution found within tolerance limit";
		}
		else if (solstat == CPXMIP_TIME_LIM_FEAS) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_FEAS) {
			solution_secondstage_exists = true;
			solution_info_secondstage = "Tree memory limit exceeded";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible";
		}
		else if (solstat == CPXMIP_INFEASIBLE) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible";
		}
		else if (solstat == CPXMIP_UNBOUNDED) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is unbounded";
		}
		else if (solstat == CPXMIP_INForUNBD) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Problem is infeasible or unbounded";
		}
		else if (solstat == CPXMIP_TIME_LIM_INFEAS) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Time limit exceeded";
		}
		else if (solstat == CPXMIP_MEM_LIM_INFEAS) {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Tree memory limit exceeded";
		}
		else {
			solution_secondstage_exists = false;
			solution_info_secondstage = "Other reason for failure";
		}

		std::cout << "\nCPLEX has finished: " << solution_info_secondstage << "\n";


		if (solution_secondstage_exists)
		{
			// for which timeslots does this decomposed problem need to be constructed
			int first_timeslot = timeslot, last_timeslot = timeslot;

			// 1. travel times
			{
				const int index_travels = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths;
				solution_travel_times.reserve(last_timeslot - first_timeslot);
				for (int t = first_timeslot; t < last_timeslot; ++t)
				{
					double time = solution_CPLEX[index_travels + t];
					solution_travel_times.push_back(time);
				}
			}

			// 2. evacuation times
			{
				const int index_evacs = nb_sessions * nb_locations + nb_timeslots * nb_series * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_series * nb_paths
					+ nb_timeslots + nb_timeslots * nb_sessions * nb_paths + nb_timeslots * nb_arcs + nb_timeslots * nb_sessions * nb_paths;
				solution_evacuation_times.reserve(last_timeslot - first_timeslot + 1);
				for (int t = first_timeslot; t <= last_timeslot; ++t)
				{
					double time = solution_CPLEX[index_evacs + t];
					solution_evacuation_times.push_back(time);
				}
			}
		}
	}



	void MIP_two_stage::run_only_first_stage_and_evaluate_solution()
	{
		auto start_time = std::chrono::system_clock::now();

		initialize_cplex();

		build_masterproblem();
		solve_masterproblem();
		clear_masterproblem();

		computation_time_firststage = std::chrono::system_clock::now() - start_time;
		start_time = std::chrono::system_clock::now();

		build_subproblem();
		fix_solution(solution_lecture_room);
		solve_subproblem();

		clear_cplex();

		computation_time_secondstage = std::chrono::system_clock::now() - start_time;

		write_output();
	}


	void MIP_two_stage::fix_solution(const std::vector<int> lecture_room)
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
		matind = std::make_unique<int[]>(10000000);
		matval = std::make_unique<double[]>(10000000);

		// Fix w_lc variables
		for (int l = 0; l < nb_sessions; ++l)
		{
			for (int c = 0; c < nb_locations; ++c)
			{
				sense[0] = 'E';

				if (lecture_room.at(l) == c)
					rhs[0] = 1;
				else
					rhs[0] = 0;

				std::string name = "Fix_w_" + std::to_string(l + 1) + "_" + std::to_string(c + 1);
				rowname[0] = const_cast<char*>(name.c_str());

				matbeg[0] = 0;
				f = 0;

				matind[f] = l * nb_locations + c;
				matval[f] = 1;
				++f;

				status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
				if (status != 0)
				{
					CPXgeterrorstring(env, status, error_text);
					throw std::runtime_error("Error in function MIP_two_stage::fix_solution(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
				}
			}
		}


		// Write to file
		status = CPXwriteprob(env, subproblem, "MIP_model_second_stage.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::build_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}
	}




	void MIP_two_stage::clear_subproblem()
	{
		int status = 0;

		// Free the subproblem
		status = CPXfreeprob(env, &subproblem);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::clear_subproblem(). \nCouldn't free the subproblem. \nReason: " + std::string(error_text));
		}
	}




	void MIP_two_stage::clear_cplex()
	{
		int status = 0;

		// Close the cplex environment
		status = CPXcloseCPLEX(&env);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::clear_cplex(). \nCouldn't close CPLEX environment. \nReason: " + std::string(error_text));
		}
	}


	void MIP_two_stage::write_output()
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
		text += "\nSolution status:";
		text += "\n\tFirst stage: " + solution_info_firststage;
		text += "\n\tSecond stage: " + solution_info_secondstage;
		text += "\nElapsed time (seconds):";
		text += "\n\tFirst stage: " + std::to_string(get_computation_time_firststage());
		text += "\n\tSecond stage: " + std::to_string(get_computation_time_secondstage());
		text += "\nModel info:";
		text += "\n\tNumber of constraints first stage: " + std::to_string(nb_constraints_firststage);
		text += "\n\tNumber of variables first stage: " + std::to_string(nb_variables_firststage);
		text += "\n\tNumber of constraints second stage: " + std::to_string(nb_constraints_secondstage);
		text += "\n\tNumber of variables second stage: " + std::to_string(nb_variables_secondstage);


		// 4. results
		text += "\nObjective value: ";
		text += "\n\tFirst stage: " + std::to_string(objective_value_firststage);
		text += "\n\tSecond stage: " + std::to_string(objective_value_secondstage);
		for (int t = 0; t < nb_timeslots; ++t)
		{
			if (_alpha > 0.01 && t < solution_evacuation_times.size())
				text += "\n\tEvacuation time timeslot " + std::to_string(t + 1) + ": " + std::to_string(solution_evacuation_times.at(t));
			if (_alpha < 0.99 && t < solution_travel_times.size())
				text += "\n\tTravel time timeslot " + std::to_string(t + 1) + ": " + std::to_string(solution_travel_times.at(t));
		}


		// 5. solution
		/*text += "\nSolution:";
		for (int t = 0; t < nb_timeslots; ++t)
		{
		text += "\nTimeslot: " + std::to_string(t + 1);
		for (int l = 0; l < nb_sessions; ++l)
		{
		if(solution_lecture_timeslot.at(l) == t)
		text += "\n\tLecture " + std::to_string(l + 1) + " assigned to room " + location_names.at(solution_lecture_room.at(l));
		}
		}*/


		// Write single item to logger
		_logger << logger::log_type::INFORMATION << text;
	}



	void MIP_two_stage::add_constraint_maximum_evacuation_time(double value)
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
			matind[f] = CPXgetnumcols(env, subproblem) - 1;
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

		status = CPXaddrows(env, subproblem, 0, 1, f, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::add_constraint_maximum_evacuation_time(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
		}


		// Write to file
		status = CPXwriteprob(env, subproblem, "MIP_model_second_stage.lp", NULL);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function MIP_two_stage::add_constraint_maximum_evacuation_time(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
		}
	}


}

