/*!
*	@file		logger.h
*	@author     Hendrik Vermuyten
*	@brief		The specification of a message logger, such that all messages to the system get properly recorded.
*/

#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <string>

/*!
*	@namespace	alg
*	@brief	The main namespace containing all algorithm elements.
*/
namespace alg
{
	/*!
	*	@brief		An html logger - writes messages to a formatted html file.
	*/
	class logger
	{
	public:
		/*!
		*	@brief		Classfies interpretation of the subsequent
		*				streaming elements.
		*/
		enum class log_type {
			UNDEFINED,		///< No interpretation provided.
			INFORMATION,    ///< Benign information - reports status.
			WARNING,		///< Error encountered and handled.
			CRITICAL   		///< Error encountered but *not* handled.
		};

		/*!
		*	@brief		Default constructor.
		*/
		logger();

		/*!
		*	@brief		Destructor.
		*/
		~logger();

		/*!
		*	@brief		Closes the logger down.
		*/
		void close();

		/*!
		*	@brief		Set the verbose status of the logger.
		*  @param      active      True if the logger is verbose, false otherwise.
		*/
		void set_verbose(bool verbose) { _verbose = verbose; }

		/*!
		*	@brief		Indicates whether the logger is verbose.
		*  @returns    True if the logger is verbose, false otherwise.
		*/
		bool is_verbose() { return _verbose; }

		/*!
		*	@brief		Writes a solid line to the logger.
		*/
		void line();

		/*!
		*	@brief		Sets the logger's output file.
		*
		*	@param		file_name		The file name to write the html to.
		*/
		void set_file(const std::string & file_name);

		/*!
		*	@brief		Writes a std::string to the logger based on current status.
		*
		*	@param		l		A reference to a logger.
		*	@param		msg			The string to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, const std::string & msg);

		/*!
		*	@brief		Writes strings to the logger based on current status.
		*
		*	@param		l		A reference to a logger.
		*	@param		msg			The string to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, const char * msg);

		/*!
		*	@brief		Writes long unsigned int to the logger based on current status.
		*
		*	@param		l		A reference to a logger.
		*	@param		value		The value to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, long unsigned int value);

		/*!
		*	@brief		Writes a size_t to the logger based on current status. Only done in
		*				windows, as GCC complains.
		*
		*	@param		l		A reference to a logger.
		*	@param		value		The value to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, size_t value);

		/*!
		*	@brief		Writes long unsigned int to the logger based on current status.
		*
		*	@param		l		A reference to a logger.
		*	@param		value		The value to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, short unsigned int value);

		/*!
		*	@brief		Writes int to the logger based on current status.
		*
		*	@param		l		A reference to a logger.
		*	@param		value		The value to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, int value);

		/*!
		*	@brief		Writes float to the logger based on current status.
		*
		*	@param		l		A reference to a logger.
		*	@param		value		The value to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, float value);

		/*!
		*	@brief		Writes double to the logger based on current status.
		*
		*	@param		l		A reference to a logger.
		*	@param		value		The value to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, double value);

		/*!
		*	@brief		Changes the message status of the logger
		*
		*	@param		l		A reference to a logger.
		*	@param		type		The value to write.
		*	@returns	A reference to the logger streamed to.
		*/
		friend logger & operator<<(logger & l, logger::log_type type);

	protected:
		/*!
		*	@brief		Writes the html header information to the given file.
		*/
		void write_header();

		/*!
		*	@brief		Writes the html tail information to the given file.
		*/
		void write_tail();

		/*!
		*	@brief		Process text.
		*
		*	Modifies the string in place to make an html version of the text if the output
		*	file is valid, otherwise, simply returns leaving the string untouched.
		*
		*	@param		input		The input text to write to the log.
		*/
		void process_text(std::string & input);

		/*!
		*	@brief		Indicates if the output file is valid.
		*/
		bool _valid_file;

		/*!
		*	@brief		The file object for the html to be written to.
		*/
		std::ofstream _file;

		/*!
		*	@brief		The current message type.
		*/
		log_type _stream_type;

		/*!
		*	@brief		Indicates whether the logger is verbose.
		*/
		bool _verbose;
	};



	/*!
	*  @brief The single globally available Logger
	*/
	extern logger _logger;

}   // namespace alg

#endif // LOGGER_H