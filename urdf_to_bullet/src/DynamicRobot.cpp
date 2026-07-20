#include "DynamicRobot.h"
#include "urdf_to_bullet/urdf_parser.h"
#include <stack>

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
	root_ = nullptr;
	assignRoot();
	if(root_ != nullptr)
	{
		printf("Identified %s as root\n", root_->name.c_str());
	}
}

void DynamicRobot::assignRoot(void)
{
	for(size_t i = 0; i < links_.size(); i++)
	{
		bool hasParent = false;
		Link * curlink = links_[i];
		for(size_t j = 0; j < curlink->joints.size(); j++)
		{
			Joint * curjoint = curlink->joints[i];
			if(curjoint->childLink == curlink)
			{
				hasParent = true;
				break;
			}
		}
		if(hasParent == false)
		{
			root_ = curlink;
			return;
		}
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




void DynamicRobot::traverse_tree_dfs(void)
{
	std::stack<Link*> stack;
	stack.push(root_);

	while(!stack.empty())
	{
		Link * cur = stack.top();
		stack.pop();

		if(cur == NULL)
		{
			return;
		}
		printf("Current node: %s\n", cur->name.c_str());

		for(size_t joint_idx = 0; joint_idx < cur->joints.size(); joint_idx++)
		{
			Joint * joint = cur->joints[joint_idx];
			printf("    has joint %s\n", joint->name.c_str());
		}
	}
}


