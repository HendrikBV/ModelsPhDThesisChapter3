/*!
*	@file	timetable_data.h
*	@author		Hendrik Vermuyten
*	@brief	A set of global variables related to the timetable information.
*/

#ifndef TIMETABLE_DATA_H
#define TIMETABLE_DATA_H

#include <vector>
#include <string>

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	/*!
	*	@brief		Indicates whether data have been imported.
	*/
	extern bool data_timetable_exist;

	/*!
	*	@brief		The name of the timetable instance.
	*/
	extern std::string instance_name_timetable;

	/*!
	*	@brief	The number of sessions (events) in the dataset.
	*/
	extern int nb_sessions;

	/*!
	*	@brief	The number of timeslots in the dataset.
	*/
	extern int nb_timeslots;

	/*!
	*	@brief	The number of timeslots per day in the dataset.
	*/
	extern int nb_timeslots_per_day;

	/*!
	*	@brief	The number of days in the dataset.
	*/
	extern int nb_days;

	/*!
	*	@brief	The number of locations (rooms) in the dataset.
	*/
	extern int nb_locations;

	/*!
	*	@brief	The number of series (curricula) in the dataset.
	*/
	extern int nb_series;

	/*!
	*	@brief	The number of teachers in the dataset.
	*/
	extern int nb_teachers;



	/*!
	*	@brief	The names of the sessions (lectures) in the dataset.
	*/
	extern std::vector<std::string> session_names;

	/*!
	*	@brief	The names of the series (curricula) in the dataset.
	*/
	extern std::vector<std::string> series_names;

	/*!
	*	@brief	The number of locations (rooms) in the dataset.
	*/
	extern std::vector<std::string> location_names;

	/*!
	*	@brief	Indicates whether a session can be assigned to a certain location.
	*/
	extern std::vector<bool> session_location_possible;

	/*!
	*	@brief	Indicates whether a series attends a session.
	*/
	extern std::vector<bool> series_session;

	/*!
	*	@brief	The number of people in each session.
	*/
	extern std::vector<int> session_nb_people;

	/*!
	*	@brief	Indicates the number of people in a certain series (curriculum).
	*/
	extern std::vector<int> series_nb_people;

	/*!
	*	@brief	The penalty value for assigning a session to a timeslot (teacher preferences).
	*/
	extern std::vector<int> cost_session_timeslot;

	/*!
	*	@brief	The penalty value for assigning a session to a timeslot (educational preferences).
	*/
	extern std::vector<int> cost_session_ts_educational;

	/*!
	*	@brief	Indicates whether a teacher teaches a certain session.
	*/
	extern std::vector<int> teacher_session;

	/*!
	*	@brief	Indicates whether two sessions have a scheduling conflict.
	*/
	extern std::vector<int> session_session_conflict;

	/*!
	*	@brief	The type of education for each series.
	*/
	extern std::vector<int> series_typeofeducation;



	/*!
	*	@brief	Returns whether the session can be scheduled in the given room.
	*	@param	session	The given session.
	*	@param	room	The given room.
	*	@returns	True if the session can be scheduled in the given room, false otherwise.
	*/
	extern bool get_sessionlocationpossible(int session, int room);

	/*!
	*	@brief	Returns whether a series of students attends a given session.
	*	@param	series	The given series.
	*	@param	session	The given session.
	*	@returns	True if the series of students attends the given session, false otherwise.
	*/
	extern bool get_seriessession(int series, int session);

	/*!
	*	@brief	Returns the cost w.r.t. teacher preferences of assigning a session to a given timeslot.
	*	@param	session	The given session.
	*	@param	timeslot	The given timeslot.
	*	@returns	The cost w.r.t. teacher preferences of assigning a session to a given timeslot.
	*/
	extern int get_costsessiontimeslot(int session, int timeslot);

	/*!
	*	@brief	Returns the cost w.r.t. educational preferences of assigning a session to a given timeslot.
	*	@param	session	The given session.
	*	@param	timeslot	The given timeslot.
	*	@returns	The cost w.r.t. educational preferences of assigning a session to a given timeslot.
	*/
	extern int get_costsession_ts_educational(int session, int timeslot);

	/*!
	*	@brief	Returns whether two sessions have a scheduling conflict.
	*	@param	session1	The first session.
	*	@param	session2	The second timeslot.
	*	@returns	True if session1 and session2 have a scheduling conflict, false otherwise.
	*/
	extern bool get_sessionsessionconflict(int session1, int session2);

	/*!
	*	@brief	Returns whether a given teacher teaches a given session.
	*	@param	teacher	The given teacher.
	*	@param	session	The given session.
	*	@returns	True if the teacher teaches the session, false otherwise.
	*/
	extern bool get_teachersession(int teacher, int session);



	/*!
	*	@brief	Read in the timetable data.
	*	@param	file_name	The name of the input file with the timetable data.
	*/
	extern void read_timetable_data(const std::string& file_name);

	/*!
	*	@brief	Delete the timetable data.
	*/
	extern void clear_timetable_data();

} // namespace alg



#endif // !TIMETABLE_DATA_H
