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
		// spawnPosition places root_ (and, transitively, the whole tree) in
		// the world; buildBulletRobot() reads it when constructing bodies.
		DynamicRobot(const std::string & path, const btVector3 & spawnPosition = btVector3(0, 0, 0));
		~DynamicRobot();
		void traverse_tree_dfs(void);	//test
		void buildBulletRobot(btDiscreteDynamicsWorld * world);
	private:
		std::vector<Link*> links_;
		std::vector<Joint*> joints_;
		Link * root_ = nullptr;
		BuildResult buildResult_;
		btTransform rootTransform_ = btTransform::getIdentity();

		void addLink(const XMLElement * link);
		void addJoint(const XMLElement * xml_joint);
		void assignRoot(void);
		void assignJoints(void);

};



