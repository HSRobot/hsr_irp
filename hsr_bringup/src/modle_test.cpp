#include <iostream>
#include <fstream>

int main(){

	std::ifstream in("some.file");
	std::ostringstream tmp;
	tmp << in.rdbuf();
	std::string str = tmp.str();
}
