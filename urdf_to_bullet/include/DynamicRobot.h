#include <string>
#include <vector>
#include <tinyxml2.h>
#include "urdf_to_bullet/urdf_model.h"

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using namespace urdf;

class DynamicRobot
{
	public:
		std::string name;
		XMLDocument doc;
		DynamicRobot(const std::string & path);
		~DynamicRobot();
		void traverse_tree_dfs(void);	//test
	private:
		std::vector<Link*> links_;
		std::vector<Joint*> joints_;
		Link * root_ = nullptr;

		void addLink(const XMLElement * link);
		void addJoint(const XMLElement * xml_joint);
		void assignRoot(void);

};



