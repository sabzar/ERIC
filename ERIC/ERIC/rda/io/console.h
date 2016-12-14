
#ifndef RDA_CONSOLE_H
#define RDA_CONSOLE_H

#include <string>
#include <map>

namespace rda {

	class Console {
	
	private :
	
		static std::map<std::string, std::string> params_;		
	
	public:
	
		static void readArgs(int argc, char* args[]);
	
		static std::string getParam(const std::string param);
	};

}

#endif