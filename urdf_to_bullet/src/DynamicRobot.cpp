#include "DynamicRobot.h"

DynamicRobot::DynamicRobot(const std::string & path)
{
	if(doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		printf("Failed to load document %s\n", path.c_str());
	}
	else
	{
		printf("Successfully loaded document: %s\n", path.c_str());
	}
}

