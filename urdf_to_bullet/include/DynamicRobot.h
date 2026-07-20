#include <string>
#include <vector>
#include <tinyxml2.h>
#include "urdf_to_bullet/urdf_model.h"
#include "urdf_to_bullet/urdf_to_bullet.h"

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
		void buildBulletRobot(btDiscreteDynamicsWorld * world);
	private:
		std::vector<Link*> links_;
		std::vector<Joint*> joints_;
		Link * root_ = nullptr;
		BuildResult buildResult_;

		void addLink(const XMLElement * link);
		void addJoint(const XMLElement * xml_joint);
		void assignRoot(void);
		void assignJoints(void);

};



