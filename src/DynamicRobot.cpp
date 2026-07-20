#include "DynamicRobot.h"
#include "urdf_to_bullet/urdf_parser.h"
#include <stack>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>

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

DynamicRobot::DynamicRobot(const std::string & path, const btVector3 & spawnPosition,
                           const std::string & meshBaseDir, double collisionDensity)
	: meshBaseDir_(meshBaseDir), collisionDensity_(collisionDensity)
{
	rootTransform_.setOrigin(spawnPosition);

	if(doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
	{
		printf("Failed to load document %s\n", path.c_str());
		return;
	}
	printf("Successfully loaded document: %s\n", path.c_str());

	XMLElement * root = doc.RootElement();
	if(!root || std::string(root->Name()) != "robot")
	{
		printf("improper urdf type\n");
		return;
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
	assignJoints();
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
			Joint * curjoint = curlink->joints[j];
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

void DynamicRobot::assignJoints(void)
{
	for(int j = 0; j < joints_.size(); j++)
	{
		Joint * curjoint = joints_[j];
		Link * parent = findLinkByName(links_, curjoint->parentLink->name);
		Link * child = findLinkByName(links_, curjoint->childLink->name);
		
		printf("Joint %s connects %s to %s\n", curjoint->name.c_str(), parent->name.c_str(), child->name.c_str());
		parent->joints.push_back(curjoint);
		child->joints.push_back(curjoint);
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

// A set of links welded together by a chain of Fixed joints, destined to
// become a single btRigidBody. `members[k].second` is that member link's
// frame relative to `members[0]` (the entry link this supernode's boundary
// joint reached, or root_ for the one supernode with no boundary joint at
// all) — identity for members[0] itself, composed via joint origins for
// everything reached through it by a Fixed joint.
struct SuperLink
{
	std::vector<std::pair<Link *, btTransform>> members;
	size_t parentSupernodeIdx = (size_t)-1;   // index into the supernodes vector; -1 for the root's supernode
	Joint * parentBoundaryJoint = nullptr;    // the non-Fixed joint connecting to the parent supernode; null for the root's supernode
	btTransform worldFrame;                   // members[0]'s frame, in world space
	btRigidBody * body = nullptr;             // set once this supernode's rigid body is built
};

// One entry on the grouping DFS stack: carries which supernode `link` belongs
// to and its frame relative to that supernode's entry link directly, so
// there's no need to search supernodes for it after popping (same idiom as
// buildRobot()'s PendingLink carrying parentBody/incomingJoint).
struct PendingWeldLink
{
	Link * link;
	btTransform worldFrame;
	btTransform localFrame;   // relative to this link's supernode's entry link
	size_t supernodeIdx;
};

void DynamicRobot::buildBulletRobot(btDiscreteDynamicsWorld * world)
{
	if(root_ == NULL)
	{
		return;
	}

	// Phase 1: group every link into a supernode by walking Fixed joints as
	// "still the same body" and every other joint type as "starts a new
	// body". Same tree walk as traverse_tree_dfs, just carrying more state.
	std::vector<SuperLink> supernodes;
	supernodes.push_back(SuperLink{});
	supernodes[0].members.push_back({root_, btTransform::getIdentity()});
	supernodes[0].worldFrame = rootTransform_;

	std::stack<PendingWeldLink> stack;
	stack.push({root_, rootTransform_, btTransform::getIdentity(), 0});

	while(!stack.empty())
	{
		PendingWeldLink cur = stack.top();
		stack.pop();

		for(size_t joint_idx = 0; joint_idx < cur.link->joints.size(); joint_idx++)
		{
			Joint * joint = cur.link->joints[joint_idx];
			if(joint->parentLink != cur.link)
			{
				continue; // only follow edges down to children
			}

			btTransform childWorldFrame = cur.worldFrame * toBtTransform(joint->origin);

			if(joint->type == JointType::Fixed)
			{
				btTransform childLocalFrame = cur.localFrame * toBtTransform(joint->origin);
				supernodes[cur.supernodeIdx].members.push_back({joint->childLink, childLocalFrame});
				stack.push({joint->childLink, childWorldFrame, childLocalFrame, cur.supernodeIdx});
			}
			else
			{
				supernodes.push_back(SuperLink{});
				size_t childIdx = supernodes.size() - 1;
				supernodes[childIdx].parentSupernodeIdx = cur.supernodeIdx;
				supernodes[childIdx].parentBoundaryJoint = joint;
				supernodes[childIdx].worldFrame = childWorldFrame;
				supernodes[childIdx].members.push_back({joint->childLink, btTransform::getIdentity()});
				stack.push({joint->childLink, childWorldFrame, btTransform::getIdentity(), childIdx});
			}
		}
	}

	printf("Welded %zu link(s) into %zu supernode(s)\n", links_.size(), supernodes.size());

	// Phase 2: one btRigidBody (with one btCompoundShape combining every
	// member link's <collision> primitives) per supernode, and one
	// btTypedConstraint per boundary joint. Processing supernodes in index
	// order is safe because a supernode's index is only ever allocated while
	// walking its already-indexed parent, so supernodes[i].parentSupernodeIdx
	// < i always — the parent body already exists by the time we reach i.
	buildResult_ = BuildResult{};

	for(size_t i = 0; i < supernodes.size(); i++)
	{
		SuperLink & sn = supernodes[i];

		bool hasCollision = false;
		for(size_t m = 0; m < sn.members.size(); m++)
		{
			if(!sn.members[m].first->collisions.empty())
			{
				hasCollision = true;
				break;
			}
		}

		btCollisionShape * shape;
		double mass = 0.0;
		btVector3 localInertia(0, 0, 0);
		btTransform principal = btTransform::getIdentity(); // entry-link frame -> true body-local origin

		if(!hasCollision)
		{
			// No compound children to attach a mass to, so there's nothing
			// for calculatePrincipalAxisTransform to compose: fall back to a
			// plain sum of whatever <inertial> the members carry (no COM
			// shift, no density — there's no collision volume for density to
			// apply to). A member with <inertial> but no <collision> of its
			// own always lands here, which is exactly why the warning below
			// exists for the mixed case (this supernode DOES have collision
			// geometry, just not on that particular member).
			btEmptyShape * empty = new btEmptyShape();
			buildResult_.shapes.push_back(empty);
			shape = empty;

			for(size_t m = 0; m < sn.members.size(); m++)
			{
				const Inertial & inertial = sn.members[m].first->inertial;
				if(inertial.present)
				{
					mass += inertial.mass;
					localInertia += btVector3((btScalar)inertial.ixx, (btScalar)inertial.iyy, (btScalar)inertial.izz);
				}
			}
		}
		else
		{
			btCompoundShape * compound = new btCompoundShape();
			buildResult_.shapes.push_back(compound);

			// One mass per compound child, same order as addChildShape
			// below — calculatePrincipalAxisTransform() needs it to find the
			// true mass-weighted COM/principal axes across every welded
			// member, rather than assuming the entry link's frame is the
			// body's local origin. Deliberately unconditional: an explicit
			// <inertial> mass is honored, but its ixx/iyy/izz tensor is not —
			// calculatePrincipalAxisTransform always re-derives each child's
			// own inertia from its geometry assuming uniform density.
			// Composing an explicit tensor by hand is a separate piece of
			// work, not needed yet since nothing currently specifies one.
			std::vector<btScalar> childMasses;

			for(size_t m = 0; m < sn.members.size(); m++)
			{
				Link * member = sn.members[m].first;
				const btTransform & memberFrame = sn.members[m].second;

				if(member->inertial.present && member->collisions.empty())
				{
					printf("Warning: link %s has <inertial> mass %.6f but no <collision> geometry of its own; "
					       "this mass has no compound child to attach to and is dropped from its supernode's "
					       "center-of-mass/inertia\n", member->name.c_str(), member->inertial.mass);
				}

				double memberVolume = 0.0;
				for(size_t c = 0; c < member->collisions.size(); c++)
				{
					memberVolume += geometryVolume(member->collisions[c].geometry);
				}

				for(size_t c = 0; c < member->collisions.size(); c++)
				{
					btCollisionShape * child = buildPrimitiveShape(member->collisions[c].geometry, buildResult_.shapes);
					compound->addChildShape(memberFrame * toBtTransform(member->collisions[c].origin), child);

					double primVolume = geometryVolume(member->collisions[c].geometry);
					double primMass = 0.0;
					if(member->inertial.present)
					{
						// Split this member's own mass across its own
						// primitives by volume share; an even split only if
						// that share can't be computed (degenerate/zero volume).
						primMass = memberVolume > 0.0
							? member->inertial.mass * (primVolume / memberVolume)
							: member->inertial.mass / (double)member->collisions.size();
					}
					else if(collisionDensity_ > 0.0)
					{
						primMass = collisionDensity_ * primVolume;
					}
					childMasses.push_back((btScalar)primMass);
				}
			}

			for(size_t c = 0; c < childMasses.size(); c++)
			{
				mass += childMasses[c];
			}

			compound->calculatePrincipalAxisTransform(childMasses.data(), principal, localInertia);
			for(int c = 0; c < compound->getNumChildShapes(); c++)
			{
				compound->updateChildTransform(c, principal.inverse() * compound->getChildTransform(c), true);
			}

			shape = compound;
		}

		// principal is the offset from the entry link's frame (sn.worldFrame)
		// to the body's true local origin — identity when there's no
		// collision geometry to compose from, otherwise the mass-weighted
		// COM/principal-axis frame calculatePrincipalAxisTransform found.
		// Every transform below that used to be "relative to the entry
		// link" now has to be re-expressed relative to that true origin.
		btTransform trueWorldFrame = sn.worldFrame * principal;
		btDefaultMotionState * motionState = new btDefaultMotionState(trueWorldFrame);
		btRigidBody::btRigidBodyConstructionInfo rbInfo((btScalar)mass, motionState, shape, localInertia);
		sn.body = new btRigidBody(rbInfo);
		world->addRigidBody(sn.body);
		buildResult_.bodies.push_back(sn.body);

		for(size_t m = 0; m < sn.members.size(); m++)
		{
			Link * member = sn.members[m].first;
			const btTransform & memberFrame = sn.members[m].second;
			buildResult_.bodiesByLinkName.push_back({member->name, sn.body});

			for(size_t v = 0; v < member->visuals.size(); v++)
			{
				VisualInstance vi;
				vi.body = sn.body;
				vi.geometry = member->visuals[v].geometry;
				vi.material = member->visuals[v].material;
				vi.localTransform = principal.inverse() * memberFrame * toBtTransform(member->visuals[v].origin);
				buildResult_.visuals.push_back(std::move(vi));
			}

			for(size_t c = 0; c < member->collisions.size(); c++)
			{
				CollisionInstance ci;
				ci.body = sn.body;
				ci.geometry = member->collisions[c].geometry;
				ci.localTransform = principal.inverse() * memberFrame * toBtTransform(member->collisions[c].origin);
				buildResult_.collisions.push_back(std::move(ci));
			}
		}

		if(sn.parentBoundaryJoint != nullptr)
		{
			btRigidBody * parentBody = supernodes[sn.parentSupernodeIdx].body;
			btTransform frameInA = parentBody->getWorldTransform().inverse() * sn.worldFrame;
			btTransform frameInB = principal.inverse();
			btTypedConstraint * constraint = makeJointConstraint(*sn.parentBoundaryJoint, *parentBody, *sn.body, frameInA, frameInB);
			world->addConstraint(constraint, /*disableCollisionsBetweenLinkedBodies=*/true);
			buildResult_.constraints.push_back(constraint);
		}
	}

	render_.emplace(buildResult_, meshBaseDir_);
}

void DynamicRobot::render(Shader & shader) const
{
	if(render_)
	{
		render_->drawVisual(shader);
	}
}

void DynamicRobot::renderCollision(Shader & shader) const
{
	if(render_)
	{
		render_->drawCollision(shader);
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
			if(joint->parentLink == cur)
			{
				stack.push(joint->childLink);
			}
			else if(joint->childLink == cur)
			{
				//skip
			}
			else
			{
				return;
			}
		}
	}
}


