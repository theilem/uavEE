//
// Created by seedship on 2/11/21.
//

#ifndef UAVEE_XPLANEINTERFACEPARAMS_H
#define UAVEE_XPLANEINTERFACEPARAMS_H

#include <cpsCore/Configuration/Parameter.hpp>

struct XPlaneInterfaceParams
{
	Parameter<std::string> log_directory = {"/tmp", "log_directory", false};
	Parameter<std::string> name = {"uavEElog-", "prepend_name", false};
	Parameter<std::string> file_extension = {".csv", "file_extension", false};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & log_directory;
		c & name;
		c & file_extension;
	}
};

#endif //UAVEE_XPLANEINTERFACEPARAMS_H
