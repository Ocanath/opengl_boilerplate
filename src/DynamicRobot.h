#include <string>
#include <vector>
#include <optional>
#include <tinyxml2.h>
#include "urdf_to_bullet/urdf_model.h"
#include "urdf_to_bullet/urdf_to_bullet.h"
#include "urdf_render.h"

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
		// meshBaseDir is prepended to <mesh filename="..."> paths when
		// render_ is built, same convention as UrdfRender itself.
		DynamicRobot(const std::string & path, const btVector3 & spawnPosition = btVector3(0, 0, 0), const std::string & meshBaseDir = "");
		~DynamicRobot();
		void traverse_tree_dfs(void);	//test
		void buildBulletRobot(btDiscreteDynamicsWorld * world);

		// <visual> geometry and <collision> primitives respectively, each
		// following its supernode's rigid body. Both are no-ops until
		// buildBulletRobot() has run — that's what builds render_.
		void render(Shader & shader) const;
		void renderCollision(Shader & shader) const;
	private:
		std::vector<Link*> links_;
		std::vector<Joint*> joints_;
		Link * root_ = nullptr;
		BuildResult buildResult_;
		btTransform rootTransform_ = btTransform::getIdentity();
		std::string meshBaseDir_;
		std::optional<UrdfRender> render_;

		void addLink(const XMLElement * link);
		void addJoint(const XMLElement * xml_joint);
		void assignRoot(void);
		void assignJoints(void);

};



