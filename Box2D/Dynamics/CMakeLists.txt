# cmake for Dynamics

project(Dynamics)

set(Dynamics_SRC
  "b2Body.cpp"
  "b2ContactManager.cpp"
  "b2Fixture.cpp"
  "b2Island.cpp"
  "b2World.cpp"
  "b2WorldCallbacks.cpp"
  )

set(Contacts_SRC
  "Contacts/b2ChainAndCircleContact.cpp"
  "Contacts/b2ChainAndPolygonContact.cpp"
  "Contacts/b2CircleContact.cpp"
  "Contacts/b2Contact.cpp"
  "Contacts/b2ContactSolver.cpp"
  "Contacts/b2EdgeAndCircleContact.cpp"
  "Contacts/b2EdgeAndPolygonContact.cpp"
  "Contacts/b2PolygonAndCircleContact.cpp"
  "Contacts/b2PolygonContact.cpp"
  )

set(Joints_SRC
  "Joints/b2DistanceJoint.cpp"
  "Joints/b2FrictionJoint.cpp"
  "Joints/b2GearJoint.cpp"
  "Joints/b2Joint.cpp"
  "Joints/b2MotorJoint.cpp"
  "Joints/b2MouseJoint.cpp"
  "Joints/b2PrismaticJoint.cpp"
  "Joints/b2PulleyJoint.cpp"
  "Joints/b2RevoluteJoint.cpp"
  "Joints/b2RopeJoint.cpp"
  "Joints/b2WeldJoint.cpp"
  "Joints/b2WheelJoint.cpp"
  )

add_library(${PROJECT_NAME} OBJECT ${Dynamics_SRC} ${Contacts_SRC} ${Joints_SRC})
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
