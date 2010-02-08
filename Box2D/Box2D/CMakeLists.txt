set(Collision
	Collision/b2BroadPhase.cpp
	Collision/b2BroadPhase.h
	Collision/b2CollideCircle.cpp
	Collision/b2CollidePolygon.cpp
	Collision/b2Collision.cpp				
	Collision/b2Collision.h
	Collision/b2Distance.cpp
	Collision/b2Distance.h
	Collision/b2DynamicTree.cpp
	Collision/b2DynamicTree.h
	Collision/b2TimeOfImpact.cpp
	Collision/b2TimeOfImpact.h
)

set(Shapes
	Collision/Shapes/b2CircleShape.cpp
	Collision/Shapes/b2CircleShape.h
	Collision/Shapes/b2PolygonShape.cpp
	Collision/Shapes/b2PolygonShape.h
	Collision/Shapes/b2Shape.h
)

set(Common
	Common/b2BlockAllocator.cpp
	Common/b2BlockAllocator.h
	Common/b2Math.cpp
	Common/b2Math.h
	Common/b2Settings.cpp
	Common/b2Settings.h
	Common/b2StackAllocator.cpp
	Common/b2StackAllocator.h
)

set(Dynamics
	Dynamics/b2Body.cpp
	Dynamics/b2Body.h
	Dynamics/b2ContactManager.cpp
	Dynamics/b2ContactManager.h
	Dynamics/b2Fixture.cpp
	Dynamics/b2Fixture.h
	Dynamics/b2Island.cpp
	Dynamics/b2Island.h
	Dynamics/b2TimeStep.h
	Dynamics/b2World.cpp
	Dynamics/b2World.h
	Dynamics/b2WorldCallbacks.cpp
	Dynamics/b2WorldCallbacks.h
)

set(Contacts
	Dynamics/Contacts/b2CircleContact.cpp
	Dynamics/Contacts/b2CircleContact.h
	Dynamics/Contacts/b2Contact.cpp
	Dynamics/Contacts/b2Contact.h
	Dynamics/Contacts/b2ContactSolver.cpp
	Dynamics/Contacts/b2ContactSolver.h
	Dynamics/Contacts/b2PolygonAndCircleContact.cpp
	Dynamics/Contacts/b2PolygonAndCircleContact.h
	Dynamics/Contacts/b2PolygonContact.cpp
	Dynamics/Contacts/b2PolygonContact.h
	Dynamics/Contacts/b2TOISolver.cpp
	Dynamics/Contacts/b2TOISolver.h
)

set(Joints
	Dynamics/Joints/b2DistanceJoint.cpp
	Dynamics/Joints/b2DistanceJoint.h
	Dynamics/Joints/b2FrictionJoint.cpp
	Dynamics/Joints/b2FrictionJoint.h
	Dynamics/Joints/b2GearJoint.cpp
	Dynamics/Joints/b2GearJoint.h
	Dynamics/Joints/b2Joint.cpp
	Dynamics/Joints/b2Joint.h
	Dynamics/Joints/b2LineJoint.cpp
	Dynamics/Joints/b2LineJoint.h
	Dynamics/Joints/b2MouseJoint.cpp
	Dynamics/Joints/b2MouseJoint.h
	Dynamics/Joints/b2PrismaticJoint.cpp
	Dynamics/Joints/b2PrismaticJoint.h
	Dynamics/Joints/b2PulleyJoint.cpp
	Dynamics/Joints/b2PulleyJoint.h
	Dynamics/Joints/b2RevoluteJoint.cpp
	Dynamics/Joints/b2RevoluteJoint.h
	Dynamics/Joints/b2WeldJoint.cpp
	Dynamics/Joints/b2WeldJoint.h
)
include_directories( ../ )

if(BOX2D_BUILD_SHARED)
	add_library(Box2D_shared SHARED 
		Box2D.h
		${Joints}
		${Contacts}
		${Dynamics}
		${Common}
		${Shapes}
		${Collision}
	)
	set_target_properties(Box2D_shared PROPERTIES OUTPUT_NAME "Box2D")
	set_target_properties(Box2D_shared PROPERTIES CLEAN_DIRECT_OUTPUT 1 VERSION ${BOX2D_VERSION})
endif()

if(BOX2D_BUILD_STATIC)
	add_library(Box2D STATIC 
		Box2D.h
		${Joints}
		${Contacts}
		${Dynamics}
		${Common}
		${Shapes}
		${Collision}
	)
	set_target_properties(Box2D PROPERTIES CLEAN_DIRECT_OUTPUT 1 VERSION ${BOX2D_VERSION})
endif()


if(MSVC)
	# These are used to create visual studio folders.
	source_group(Collision FILES ${Collision})
	source_group(Collision\\Shapes FILES ${Shapes})
	source_group(Common FILES ${Common})
	source_group(Dynamics FILES ${Dynamics})
	source_group(Dynamics\\Contacts FILES ${Contacts})
	source_group(Dynamics\\Joints FILES ${Joints})
	source_group(Include Box2D.h)
endif()

if(UNIX)
	# install time stuff
	install(DIRECTORY ./ DESTINATION include/Box2D
			FILES_MATCHING PATTERN "*.h"
			PATTERN ".svn" EXCLUDE)
			
	if(BOX2D_BUILD_SHARED)
		install(TARGETS Box2D_shared LIBRARY DESTINATION usr/lib)
	endif()

	if(BOX2D_BUILD_STATIC)
		install(TARGETS Box2D ARCHIVE DESTINATION usr/lib)
	endif()
endif()