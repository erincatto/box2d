# Common Module
The Common module contains settings, memory management, and vector math.

## Settings
The header b2Settings.h contains:
- Types such as int32 and float
- Constants
- Allocation wrappers
- The version number

### Types
Box2D defines various types such as int8, etc. to make it easy
to determine the size of structures.

### Constants
Box2D defines several constants. These are all documented in
b2Settings.h. Normally you do not need to adjust these constants.

Box2D uses floating point math for collision and simulation. Due to
round-off error some numerical tolerances are defined. Some tolerances
are absolute and some are relative. Absolute tolerances use MKS units.

### Allocation wrappers
The settings file defines b2Alloc and b2Free for large allocations. You
may forward these calls to your own memory management system.

### Version
The b2Version structure holds the current version so you can query this
at run-time.

## Memory Management
A large number of the decisions about the design of Box2D were based on
the need for quick and efficient use of memory. In this section I will
discuss how and why Box2D allocates memory.

Box2D tends to allocate a large number of small objects (around 50-300
bytes). Using the system heap through malloc or new for small objects is
inefficient and can cause fragmentation. Many of these small objects may
have a short life span, such as contacts, but can persist for several
time steps. So we need an allocator that can efficiently provide heap
memory for these objects.

Box2D's solution is to use a small object allocator (SOA) called
b2BlockAllocator. The SOA keeps a number of growable pools of varying
sizes. When a request is made for memory, the SOA returns a block of
memory that best fits the requested size. When a block is freed, it is
returned to the pool. Both of these operations are fast and cause little
heap traffic.

Since Box2D uses a SOA, you should never new or malloc a body, fixture,
or joint. However, you do have to allocate a b2World on your own. The
b2World class provides factories for you to create bodies, fixtures, and
joints. This allows Box2D to use the SOA and hide the gory details from
you. Never, call delete or free on a body, fixture, or joint.

While executing a time step, Box2D needs some temporary workspace
memory. For this, it uses a stack allocator called b2StackAllocator to
avoid per-step heap allocations. You don't need to interact with the
stack allocator, but it's good to know it's there.

## Math
Box2D includes a simple small vector and matrix module. This has been
designed to suit the internal needs of Box2D and the API. All the
members are exposed, so you may use them freely in your application.

The math library is kept simple to make Box2D easy to port and maintain.
