/**\file tsv_parser.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <qualisys_tsv_parsers/tsv_parser.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace qualisys_tsv_parsers {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
TSVParser::TSVParser(TSVPointType tsv_point_type) :
		load_base_time_from_tsv_header_(true),
		tsv_point_type_(tsv_point_type),
		tsv_data_multiplier_(0.001),
		tsv_data_offset_x_(0.0),
		tsv_data_offset_y_(0.0),
		tsv_data_offset_z_(0.0),
		tsv_time_multiplier_(1.0),
		tsv_null_string_("NULL")
		{}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TSVParser-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void TSVParser::setupConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	private_node_handle->param("load_base_time_from_tsv_header", load_base_time_from_tsv_header_, true);

	double time_offset = 0.0;
	private_node_handle->param("tsv_time_offset", time_offset, 0.0);
	time_offset_.fromSec(time_offset);

	std::string tsv_data_columns;
	private_node_handle->param("tsv_data_columns", tsv_data_columns, std::string("2+5+8+11+14+17+20+23"));
	parseTSVDataColumns(tsv_data_columns);

	private_node_handle->param("tsv_data_multiplier", tsv_data_multiplier_, 0.001);
	private_node_handle->param("tsv_data_offset_x", tsv_data_offset_x_, 0.0);
	private_node_handle->param("tsv_data_offset_y", tsv_data_offset_y_, 0.0);
	private_node_handle->param("tsv_data_offset_z", tsv_data_offset_z_, 0.0);

	private_node_handle->param("tsv_time_multiplier", tsv_time_multiplier_, 1.0);
	private_node_handle->param("tsv_null_string", tsv_null_string_, std::string("NULL"));
}


void TSVParser::parseTSVDataColumns(std::string& offsets) {
	std::replace(offsets.begin(), offsets.end(), '+', ' ');

	std::stringstream ss(offsets);
	size_t offset;

	while (ss >> offset) {
		tsv_data_columns_.push_back(offset);
	}
}


bool TSVParser::parseTSVHeader(std::ifstream& input_stream) {
	// time offset
	if (load_base_time_from_tsv_header_) {
		std::string tsv_base_time;
		if (!loadTSVValue(input_stream, "TIME_STAMP", tsv_base_time)) { return false; }
		base_time_stamp_.fromSec(parseUTCBaseTime(tsv_base_time));
		base_time_stamp_ += time_offset_;
	} else {
		base_time_stamp_ = ros::Time::now();
	}

	//data type
	std::string data_type;
	if (!loadTSVValue(input_stream, "DATA_INCLUDED", data_type))  { return false; }
	std::stringstream ss(data_type);
	ss >> data_type;

	if (data_type == "2D") {
		tsv_point_type_ = Point2D;
	} else if (data_type == "3D") {
		tsv_point_type_ = Point3D;
	} else if (data_type == "6D") {
		tsv_point_type_ = Point6D;
	} else {
		return false;
	}

	// discard remaining info
	std::string remaining_data;
	if (!loadTSVValue(input_stream, "Frame", remaining_data)) { return false; }

	return true;
}


bool TSVParser::loadTSVValue(std::ifstream& input_stream, const std::string& key, std::string& value_out) {
	std::string line;
	while (std::getline(input_stream, line)) {
		std::stringstream ss(line);
		std::string tsv_key;

		ss >> tsv_key;
		if (tsv_key == key) {
			std::getline(ss, value_out);
			return true;
		}
	}

	return false;
}


double TSVParser::parseUTCBaseTime(const std::string& tsv_base_time) {
	std::string temp_data;
	std::stringstream ss(tsv_base_time);
	ss >> temp_data;
	ss >> temp_data;
	double time_from_boot;
	ss >> time_from_boot;

	return time_from_boot;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TSVParser-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================


} /* namespace qualisys_tsv_parsers */

