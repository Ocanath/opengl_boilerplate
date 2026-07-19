#include <string>
#include <tinyxml2.h>

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;


class DynamicRobot
{
	public:
		std::string name;
		XMLDocument doc;
		DynamicRobot(const std::string & path);
	private:

};

