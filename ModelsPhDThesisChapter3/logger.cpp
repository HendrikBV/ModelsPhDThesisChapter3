#include "logger.h"

#include <iostream>

namespace alg
{
	/////////////////////////////////////////////////////////////////////
	///					Implementation of logger
	/////////////////////////////////////////////////////////////////////

	logger _logger;

	/////////////////////////////////////////////////////////////////////

	logger::logger() : _valid_file(false), _file(), _stream_type(log_type::UNDEFINED)
	{
		std::ofstream css_file;
		css_file.open("algorithm_log.css");
		if (css_file.is_open()) {
			css_file << "table, th, td {\n";
			css_file << "\tborder-collapse:collapse;\n";
			css_file << "\tborder: 1px solid grey;\n";
			css_file << "}\n";
			css_file << "table {\n";
			css_file << "\ttable-layout : fixed;\n";
			css_file << "\twidth: 950 px;\n";
			css_file << "\tmax-width: 900 px;\n";
			css_file << "\tbackground-color : #292929;\n";
			css_file << "}\n";
			css_file << "td {\n";
			css_file << "\tmin-width: 200px;\n";
			css_file << "}\n";
			css_file << "#content {\n";
			css_file << "\tposition : relative;\n";
			css_file << "}\n";
			css_file << "body, html {\n";
			css_file << "\tbackground: #000000;\n";
			css_file << "\twidth: 1000px;\n";
			css_file << "\tfont-family: Arial;\n";
			css_file << "\tfont-size: 16px;\n";
			css_file << "\tcolor: #C0C0C0;\n";
			css_file << "}\n";
			css_file << "h1 {\n";
			css_file << "\tfont-size : 50px;\n";
			css_file << "\tline-height : 100px;\n";
			css_file << "\tcolor : #FFFFFF;\n";
			css_file << "\tborder-bottom : 1px dotted #888888;\n";
			css_file << "}\n";
			css_file << "#logo {\n";
			css_file << "\tposition: absolute;\n";
			css_file << "\ttop: 0px;\n";
			css_file << "\tright: 0px;\n";
			css_file << "}\n";
			css_file << ".divider {\n";
			css_file << "\tbackground : #DDD;\n";
			css_file << "}\n";
			css_file << ".box {\n";
			css_file << "\tpadding : 0px;\n";
			css_file << "}\n";
			css_file << ".inf {\n";
			css_file << "\tcolor: #C0C0C0;\n";
			css_file << "\tmin-width : 1000px;\n";
			css_file << "\tmax-width : 1000px;\n";
			css_file << "}\n";
			css_file << ".err {\n";
			css_file << "\tcolor: #EE1100;\n";
			css_file << "\tfont-weight: bold\n";
			css_file << "}\n";
			css_file << ".war {\n";
			css_file << "\tcolor: #FFCC00;\n";
			css_file << "\tfont-weight: bold\n";
			css_file << "}";

			css_file.close();

			set_file("algorithm_log.html");
		}
		else {
			std::cout << "Unable to create style sheet for logger\n";
		}
	}

	/////////////////////////////////////////////////////////////////////

	logger::~logger() {
		close();
	}

	/////////////////////////////////////////////////////////////////////

	void logger::close() {
		if (_valid_file) {
			write_tail();
			_file.close();
			_valid_file = false;
		}
	}

	/////////////////////////////////////////////////////////////////////

	void logger::line() {
		if (_valid_file) {
			if (_stream_type != log_type::UNDEFINED) {
				_file << "</td>\n\t</tr>\n";
			}
			_file << "\t<tr>\n\t\t<td class=\"divider\"/>\n\t</tr>\n";
		}
		_stream_type = log_type::UNDEFINED;
	}

	/////////////////////////////////////////////////////////////////////

	void logger::set_file(const std::string & file_name) {
		_file.open(file_name);
		_valid_file = _file.is_open();
		if (_valid_file) {
			write_header();
		}
		else {
			std::cout << "Error opening file for writing a log\n";
		}
	}

	/////////////////////////////////////////////////////////////////////

	void logger::write_header() {
		_file << "<html>\n";
		_file << "<head>\n";
		_file << "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />\n";
		_file << "<title>Algorithm Log</title>\n";
		_file << "<link rel=\"stylesheet\" type=\"text/css\" href=\"algorithm_log.css\" ";
		_file << "media=\"screen\"/>\n";
		_file << "</head>\n\n";

		_file << "<body>\n";
		_file << "<div id=\"content\">\n";
		_file << "<h1>Algorithm Log</h1>\n";
		_file << "<img id=\"logo\" src=\"logo.png\"/>\n";
		_file << "<div class=\"box\">\n";
		_file << "<table>\n";
	}

	/////////////////////////////////////////////////////////////////////

	void logger::write_tail() {
		if (_stream_type != log_type::UNDEFINED) {
			_file << "</td>\n\t</tr>\n";
		}
		_file << "</table>\n";
		_file << "</div>\n";
		_file << "</div>\n";
		_file << "</body>\n";
		_file << "</html>\n";
	}

	/////////////////////////////////////////////////////////////////////

	inline void find_and_replace(std::string & source, const std::string & find, const std::string & replace)
	{
		size_t fLen = find.size();
		size_t rLen = replace.size();
		for (size_t pos = 0; (pos = source.find(find, pos)) != std::string::npos; pos += rLen) {
			source.replace(pos, fLen, replace);
		}
	}

	/////////////////////////////////////////////////////////////////////

	void logger::process_text(std::string & input) {
		if (_valid_file) {
			// tags
			find_and_replace(input, std::string("<"), std::string("&lt;"));
			find_and_replace(input, std::string(">"), std::string("&gt;"));
			// carriage returns
			find_and_replace(input, std::string("\n"), std::string("<br>"));
			// tabs
			find_and_replace(input, std::string("\t"), std::string("&nbsp;&nbsp;&nbsp;&nbsp;"));
		}
	}

	/////////////////////////////////////////////////////////////////////

	logger & operator<<(logger & l, const std::string & msg) {
		std::string msgStr(msg);
		l.process_text(msgStr);
		if (l._valid_file) {
			l._file << msgStr;
		}
		return l;
	}

	/////////////////////////////////////////////////////////////////////

	logger & operator<<(logger & l, const char * msg) {
		std::string msgStr(msg);
		l.process_text(msgStr);
		if (l._valid_file) {
			l._file << msgStr;
		}
		return l;
	}

	/////////////////////////////////////////////////////////////////////

	logger & operator<<(logger & l, long unsigned int value) {
		if (l._valid_file) {
			l._file << value;
		}
		return l;
	}

	/////////////////////////////////////////////////////////////////////

	logger & operator<<(logger & l, size_t value) {
		if (l._valid_file) {
			l._file << value;
		}
		return l;
	}

	/////////////////////////////////////////////////////////////////////

	logger & operator<<(logger & l, short unsigned int value) {
		if (l._valid_file) {
			l._file << value;
		}
		return l;
	}

	/////////////////////////////////////////////////////////////////////


	logger & operator<<(logger & l, int value) {
		if (l._valid_file) {
			l._file << value;
		}
		return l;
	}

	/////////////////////////////////////////////////////////////////////

	logger & operator<<(logger & l, float value) {
		if (l._valid_file) {
			l._file << value;
		}
		return l;
	}

	/////////////////////////////////////////////////////////////////////

	logger & operator<<(logger & l, double value) {
		if (l._valid_file) {
			l._file << value;
		}
		return l;
	}

	/////////////////////////////////////////////////////////////////////

	logger & operator<<(logger & l, logger::log_type type) {
		if (l._valid_file) {
			l._file.flush();
			if (l._stream_type != logger::log_type::UNDEFINED) {
				l._file << "</td>\n\t</tr>\n";
			}
			else {
				type = logger::log_type::INFORMATION;
			}
			l._file << "\t<tr>\n\t\t<td class=\"";
			switch (type) {
			case logger::log_type::UNDEFINED:
			case logger::log_type::INFORMATION:
				l._file << "inf";
				break;
			case logger::log_type::WARNING:
				l._file << "war";
				break;
			case logger::log_type::CRITICAL:
				l._file << "err";
				break;
			}
			l._file << "\">";
		}
		l._stream_type = type;
		return l;
	}


}