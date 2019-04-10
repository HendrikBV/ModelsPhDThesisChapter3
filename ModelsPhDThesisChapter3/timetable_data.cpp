#include "timetable_data.h"

#include <fstream>
#include <iostream>
#include <random>


namespace alg
{
	bool data_timetable_exist = false;
	std::string instance_name_timetable;

	int nb_sessions;
	int nb_timeslots;
	int nb_locations;
	int nb_series;

	int nb_teachers;
	int nb_timeslots_per_day = 0;
	int nb_days = 0;

	std::vector<std::string> session_names;
	std::vector<std::string> series_names;
	std::vector<std::string> location_names;

	std::vector<bool> session_location_possible;
	std::vector<bool> series_session;
	std::vector<int> session_nb_people;
	std::vector<int> series_nb_people;

	std::vector<int> cost_session_timeslot;
	std::vector<int> cost_session_ts_educational;
	std::vector<int> teacher_session;
	std::vector<int> series_typeofeducation;
	std::vector<int> session_session_conflict;

	bool get_sessionlocationpossible(int session, int room) { return session_location_possible[session*nb_locations + room]; }
	bool get_seriessession(int series, int session) { return series_session[series*nb_sessions + session]; }

	int get_costsessiontimeslot(int session, int timeslot) { return cost_session_timeslot[session*nb_timeslots + timeslot]; }
	int get_costsession_ts_educational(int session, int timeslot) { return cost_session_ts_educational[session*nb_timeslots + timeslot]; }
	bool get_sessionsessionconflict(int session1, int session2) { return session_session_conflict[session1*nb_sessions + session2]; }
	bool get_teachersession(int teacher, int session) { return teacher_session[teacher*nb_sessions + session]; }



	void read_timetable_data(const std::string& file_name)
	{
		std::ifstream myfile;
		myfile.open(file_name);
		if (!myfile.is_open())
			std::cerr << "\n\nError: couldn't open input file for timetable data\n\n";
		else
		{
			// clear old data
			clear_timetable_data();

			// instance name
			myfile >> instance_name_timetable;

			// first read in basic data
			myfile >> nb_sessions;
			myfile >> nb_series;
			myfile >> nb_timeslots;
			myfile >> nb_days;
			myfile >> nb_locations;
			myfile >> nb_teachers;

			if (nb_days > 0)
				nb_timeslots_per_day = nb_timeslots / nb_days;
			else
				nb_timeslots_per_day = 0;

			// read in data
			// names sessions
			session_names.reserve(nb_sessions);
			for (int l = 0; l < nb_sessions; ++l)
			{
				std::string name;
				myfile >> name;
				session_names.push_back(name);
			}

			// names series
			series_names.reserve(nb_series);
			for (int c = 0; c < nb_series; ++c)
			{
				std::string name;
				myfile >> name;
				series_names.push_back(name);
			}

			// names locations
			location_names.reserve(nb_locations);
			for (int l = 0; l < nb_locations; ++l)
			{
				std::string name;
				myfile >> name;
				location_names.push_back(name);
			}



			// series session
			series_session.reserve(nb_series*nb_sessions);
			for (int c = 0; c < nb_series; ++c)
			{
				for (int l = 0; l < nb_sessions; ++l)
				{
					int k;
					myfile >> k;
					series_session.push_back(k);
				}
			}

			// session location possible
			session_location_possible.reserve(nb_sessions*nb_locations);
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int r = 0; r < nb_locations; ++r)
				{
					int k;
					myfile >> k;
					session_location_possible.push_back(k);
				}
			}

			// session nb_people
			session_nb_people.reserve(nb_sessions);
			for (int l = 0; l < nb_sessions; ++l)
			{
				int k;
				myfile >> k;
				session_nb_people.push_back(k);
			}

			// series nb_people
			series_nb_people.reserve(nb_series);
			for (int c = 0; c < nb_series; ++c)
			{
				int k;
				myfile >> k;
				series_nb_people.push_back(k);
			}

			/////////////////

			// teacher lecture
			teacher_session.reserve(nb_teachers*nb_sessions);
			for (int i = 0; i < nb_teachers; ++i)
			{
				for (int j = 0; j < nb_sessions; ++j)
				{
					int k;
					myfile >> k;
					teacher_session.push_back(k);
				}
			}

			// cost lecture ts
			cost_session_timeslot.reserve(nb_sessions*nb_timeslots);
			for (int i = 0; i < nb_sessions; ++i)
			{
				for (int j = 0; j < nb_timeslots; ++j)
				{
					int k;
					myfile >> k;
					cost_session_timeslot.push_back(k);
				}
			}

			// series type of education
			series_typeofeducation.reserve(nb_series);
			for (int i = 0; i < nb_series; ++i)
			{
				int k;
				myfile >> k;
				series_typeofeducation.push_back(k);
			}

			// end file input
			data_timetable_exist = true;



			/////////////////

			// lecture_lecture_conflict
			session_session_conflict.reserve(nb_sessions*nb_sessions);
			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int m = 0; m < nb_sessions; ++m)
				{
					session_session_conflict.push_back(false);

					// same teacher
					for (int d = 0; d < nb_teachers; ++d)
					{
						if (get_teachersession(d, l) && get_teachersession(d, m))
						{
							session_session_conflict[l*nb_sessions + m] = true;
							break;
						}
					}

					// same curriculum
					for (int s = 0; s < nb_series; ++s)
					{
						if (get_seriessession(s, l) && get_seriessession(s, m))
						{
							session_session_conflict[l*nb_sessions + m] = true;
							break;
						}
					}
				}
			}

			// calculate cost_session_ts_educational
			cost_session_ts_educational.reserve(nb_sessions*nb_timeslots);
			for (int i = 0; i < nb_sessions*nb_timeslots; ++i)
				cost_session_ts_educational.push_back(0);

			for (int l = 0; l < nb_sessions; ++l)
			{
				for (int s = 0; s < nb_series; ++s)
				{
					if (get_seriessession(s, l))
					{
						for (int t = 0; t < nb_timeslots; ++t)
						{
							// only count incompatibilities once
							//if (get_costsession_ts_educational(l, t) >= 1)
							//	break;

							if (nb_timeslots_per_day == 5)
							{
								if (series_typeofeducation[s] == 0)
								{
									// nothing
								}
								else if (series_typeofeducation[s] == 1)
								{
									// last 3 timeslots of each day not allowed
									if (t % nb_timeslots_per_day == 2 || t % nb_timeslots_per_day == 3 || t % nb_timeslots_per_day == 4)
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
								else if (series_typeofeducation[s] == 2)
								{
									// first 3 timeslots of each day not allowed
									if (t % nb_timeslots_per_day == 0 || t % nb_timeslots_per_day == 1 || t % nb_timeslots_per_day == 2)
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
								else if (series_typeofeducation[s] == 3)
								{
									// only tuesday and thursday last 2 timeslots allowed
									if (!(((int)(t / nb_timeslots_per_day + 0.1) == 1 || (int)(t / nb_timeslots_per_day + 0.1) == 3) && (t % nb_timeslots_per_day == 3 || t % nb_timeslots_per_day == 4)))
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
							}
							else if (nb_timeslots_per_day == 6)
							{
								if (series_typeofeducation[s] == 0)
								{
									// last timeslot of each day not allowed
									if (t % nb_timeslots_per_day == 5)
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
								else if (series_typeofeducation[s] == 1)
								{
									// last 4 timeslots of each day not allowed
									if (t % nb_timeslots_per_day == 2 || t % nb_timeslots_per_day == 3 || t % nb_timeslots_per_day == 4 || t % nb_timeslots_per_day == 5)
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
								else if (series_typeofeducation[s] == 2)
								{
									// first 4 timeslots of each day not allowed
									if (t % nb_timeslots_per_day == 0 || t % nb_timeslots_per_day == 1 || t % nb_timeslots_per_day == 2 || t % nb_timeslots_per_day == 3)
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
								else if (series_typeofeducation[s] == 3)
								{
									// only last 2 timeslots of tuesday and thursday allowed
									if (!(((int)(t / nb_timeslots_per_day + 0.1) == 1 || (int)(t / nb_timeslots_per_day + 0.1) == 3) && (t % nb_timeslots_per_day == 4 || t % nb_timeslots_per_day == 5)))
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
							}
							else if (nb_timeslots_per_day == 9)
							{
								if (series_typeofeducation[s] == 0)
								{
									// last 2 timeslots of each day not allowed
									if (t % nb_timeslots_per_day == 7 || t % nb_timeslots_per_day == 8)
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
								else if (series_typeofeducation[s] == 1)
								{
									// last 6 timeslots of each day not allowed
									if (!(t % nb_timeslots_per_day == 0 || t % nb_timeslots_per_day == 1 || t % nb_timeslots_per_day == 2))
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
								else if (series_typeofeducation[s] == 2)
								{
									// first 6 timeslots of each day not allowed
									if (!(t % nb_timeslots_per_day == 6 || t % nb_timeslots_per_day == 7 || t % nb_timeslots_per_day == 8))
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
								else if (series_typeofeducation[s] == 3)
								{
									// only last 3 timeslots of tuesday and thursday allowed
									if (!(((int)(t / nb_timeslots_per_day + 0.1) == 1 || (int)(t / nb_timeslots_per_day + 0.1) == 3) && (t % nb_timeslots_per_day == 6 || t % nb_timeslots_per_day == 7 || t % nb_timeslots_per_day == 8)))
										++cost_session_ts_educational[l*nb_timeslots + t];
								}
							}
						}
					}
				}
			}
		}
	}


	void clear_timetable_data()
	{
		data_timetable_exist = false;

		instance_name_timetable = "";

		nb_sessions = 0;
		nb_series = 0;
		nb_timeslots = 0;
		nb_locations = 0;
		nb_teachers = 0;

		nb_timeslots_per_day = 0;
		nb_days = 0;

		session_names.clear();
		series_names.clear();
		location_names.clear();

		session_nb_people.clear();
		series_nb_people.clear();
		session_location_possible.clear();
		series_session.clear();

		cost_session_timeslot.clear();
		cost_session_ts_educational.clear();
		teacher_session.clear();
		session_session_conflict.clear();
		series_typeofeducation.clear();
	}



}