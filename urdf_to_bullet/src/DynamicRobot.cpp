#include "DynamicRobot.h"
#include "urdf_to_bullet/urdf_parser.h"

void DynamicRobot::addLink(const XMLElement * xml_link)
{
	Link * link = new Link;
	parseLink(xml_link, link);
	printf("Adding link %s\n", link->name.c_str());
	links_.push_back(link);
}

void DynamicRobot::addJoint(const XMLElement * xml_joint)
{
	Joint * joint = new Joint;

	parseJoint(xml_joint, links_, joint);
	printf("Added joint %s\n", joint->name.c_str());
	joints_.push_back(joint);
}

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


	XMLElement * root = doc.RootElement();
	if(!root || std::string(root->Name()) != "robot")
	{
		printf("improper urdf type\n");
	}

	name = root->Attribute("name");
	printf("robot name: %s\n", name.c_str());

	for(const XMLElement * link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
	{
		addLink(link);
	}	
	for(const XMLElement * joint = root->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
	{
		addJoint(joint);
	}	

}

DynamicRobot::~DynamicRobot()
{
	printf("Tearing down %s\n", name.c_str());
	for(size_t i = 0; i < links_.size(); i++)
	{
		delete links_[i];
	}
	for(size_t i = 0; i < joints_.size(); i++)
	{
		delete joints_[i];
	}
}













