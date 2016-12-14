
#include "console.h"
#include <iostream>

std::map<std::string, std::string> rda::Console::params_ ;

void rda::Console::readArgs(int argc, char* argv[]){

	if(argc > 1){
		for(int i = 1; i < argc - 1; i+=2){
			params_[argv[i]] = argv[i+1];
		}
	}
}

std::string rda::Console::getParam(const std::string param){
	return params_[param];
}