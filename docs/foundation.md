# Foundations

Box2D provides base functionality for hooks and vector math. The C interface allows most of the
runtime data and types to be defined internally in the `src` folder.

## Assertions

Box2D may assert on bad input. This includes things like sending in NaN or infinity for values. It
may assert if you use negative values for things that should only be positive, such as density. When
asserts are disabled, bad input may be rejected and a message may be logged.

Box2D will also assert if an internal bug is detected. For this reason, it is advisable to build
Box2D from source. The Box2D library compiles in about a second on my computer.

You may wish to capture assertions in your application. In this case you can use
`b2SetAssertCallback()`. This allows you to override the debugger break and/or perform your own
error handling.

## Allocation

Box2D uses memory efficiently and minimizes per frame allocations by pooling memory. The engine
quickly adapts to the simulation size.

As bodies, shapes, and joints are created and destroyed, their memory will be recycled. Internally
all this data is stored in contiguous arrays. When an object is destroyed, the array element will be
marked as empty. And when an object is created it will use empty slots in the array using an
efficient free list.

Once the internal memory pools are initially filled, the only allocations should be for sleeping
islands since their data is copied out of the main simulation. These allocations should be
infrequent.

You can provide a custom allocator using `b2SetAllocator()` and you can get the number of bytes
allocated using `b2GetByteCount()`.

## Logging

You can get logging from Box2D using `b2SetLogCallback()`. Logging in Box2D is basic. It is just a
substitute for `printf`. Logging is mainly used to report invalid input data.

## Version

The `b2Version` structure holds the current version so you can query this at run-time using
`b2GetVersion()`.

```c
b2Version version = b2GetVersion();
printf("Box2D version %d.%d.%d\n", version.major, version.minor, version.revision);
```

The version is also available at compile time from these macros:

```c
// Significant API and behavior changes.
#define B2_VERSION_MAJOR 3

// API changes and features.
#define B2_VERSION_MINOR 2

// Bug fixes. No API changes.
#define B2_VERSION_REVISION 0
```

## Compile time configuration

Box2D can be configured in CMake and all the settings are shown using cmake-gui. If you vendor Box2D
you can edit `box2d/config.h` to define compile time options. For example, you can enable double
precision body positions with this macro:

```c
#define BOX2D_DOUBLE_PRECISION
```

Alternatively, you can define the C preprocessor value `BOX2D_USER_CONFIG` to provide your own
configuration file. This is a convenient way to override some values in `box2d/constants.h`. Not all
values can or should be overridden.

## Vector math

Box2D includes a small vector math library including types `b2Vec2`, `b2Rot`, `b2Transform`, and
`b2AABB`. This has been designed to suit the internal needs of Box2D and the interface. All the
members are exposed, so you may use them freely in your application.

The math library is kept simple to make Box2D easy to port and maintain.

## Multithreading {#multi}

Box2D has been highly optimized for multithreading. Multithreading is not required and by default
Box2D will run single-threaded. If performance is important for your application, you should
consider using multithreading.

Box2D has a built-in task scheduler that creates threads. You can optionally connect your own task
scheduler if you want more control.

Multithreading is established for each Box2D world you create and must be configured in the world
definition. See `b2TaskCallbackFcn()`, `b2EnqueueTaskCallbackFcn()`, and `b2FinishTaskCallbackFcn()`
for more details. Also see `b2WorldDef::workerCount`, `b2WorldDef::enqueueTask`, and
`b2WorldDef::finishTask`.

You can change the worker count using `b2World_SetWorkerCount()`.

### Using the built-in task scheduler

You can use the built-in scheduler simply by setting the worker count.

```c
b2WorldDef def = b2DefaultWorldDef();

// Use 5 workers. Box2D will create 4 additional threads. Worker 0 is
// the thread that calls b2World_Step.
def.workerCount = 5;
```

### Using an external task scheduler

You can hook up an external task scheduler using callbacks. You still need to specify the worker
count. Box2D uses the worker count to allocate and access per thread data.

```c
b2WorldDef def = b2DefaultWorldDef();

// Use 5 workers. Box2D assumes you will have 4 additional threads to
// process tasks.
def.workerCount = 5;

// MyEnqueueTask is a function you implement that has the signature of
// b2EnqueueTaskCallbackFcn.
def.enqueueTask = MyEnqueueTask;

// MyFinishTask is a function you implement that has the signature of
// b2FinishTaskCallbackFcn.
def.finishTask = MyFinishTask;

// myTaskContext is a void pointer to your data that you get access to in the
// above callbacks. Typically this is a pointer to your task scheduler.
def.userTaskContext = myTaskContext;
```

The worker count is a target. If your threads are busy, Box2D can still finish its work with a
single thread.

The finish callback is optional. Without it `b2World_Step()` waits on an internal semaphore that
the last task of each phase signals. This suits task systems that can launch work but cannot wait
on a single task.

### Stepping without blocking

`b2World_Step()` holds the calling thread until the step is done. Some task systems cannot allow
this, for example a phased or wavefront scheduler where a thread that joins a parallel job cannot
leave it. For these you can drive the step yourself with `b2World_BeginStep()` and
`b2World_ContinueStep()`. Each call runs the serial work between two phases and returns the next
batch of tasks. An empty batch means the step is complete.

```c
b2TaskBatch batch = b2World_BeginStep( worldId, timeStep, subStepCount );
while ( batch.count > 0 )
{
    // Run every task in the batch on any threads, in any order, then wait for them.
    // MyRunTasks is a function you implement.
    MyRunTasks( batch.tasks, batch.count );

    batch = b2World_ContinueStep( worldId );
}
```

No callbacks are needed and the built-in scheduler is never created. The worker count still sets
the most tasks in a batch, `workerCount + 2`, and the size of the per-worker data.

The rules:

- Call each task function exactly once. Tasks in a batch may run concurrently on any threads.
- Begin and continue may be called from any thread, but never while a task of the current batch is
  running.
- The world is locked until the step completes. Do not read or write it in between.
- The tasks of the constraint solver wait on each other. They work with any number of threads,
  even when run one after another on a single thread, but a solver task that has started must not
  be suspended indefinitely.

One task may be marked `b2Task::deferred`. This is the rebuild of the dynamic tree, which overlaps
the narrow phase and the solver. You may leave it running across continue calls. A later batch
lists it in `b2TaskBatch::joinTasks` and it must be complete before the continue that follows that
batch. If you ignore the flag and run it with the rest of its batch, the step is still correct, it
just loses the overlap.

### Threading considerations

The multithreading design for Box2D is focused on [data
parallelism](https://en.wikipedia.org/wiki/Data_parallelism). The idea is to use multiple cores to
complete the world simulation as fast as possible. Box2D multithreading is not designed for [task
parallelism](https://en.wikipedia.org/wiki/Task_parallelism). Often in games you may have a render
thread and an audio thread that do work in isolation from the main thread. Those are examples of
task parallelism.

So when you design your game loop, you should let Box2D *go wide* and use multiple cores to finish
its work quickly, without other threads trying to interact with the Box2D world.

In a multithreaded environment you must be careful to avoid [race
conditions](https://en.wikipedia.org/wiki/Race_condition). Modifying the world while it is
simulating will lead to unpredictable behavior and this is never safe. It is also not safe to read
data from a Box2D world while it is simulating. Box2D may move data structures to improve cache
performance. So it is very likely that you will read garbage data.

> [!warning]
> Do not perform read or write operations on a Box2D world during `b2World_Step()`

> [!warning]
> Do not write to the Box2D world from multiple threads

It *is safe* to do ray-casts, shape-casts, and overlap tests from multiple threads outside of
`b2World_Step()`. Generally, any read-only operation is safe to do multithreaded outside of
`b2World_Step()`. This can be very useful if you have multithreaded game logic.

## Multithreading multiple worlds

Some applications may wish to create multiple Box2D worlds and simulate them on different threads.
This works fine because Box2D has very limited use of globals.

There are a few caveats:

- You will get a race condition if you create or destroy Box2D worlds from multiple threads. You
  should use a mutex to guard this.
- If you will simulate multiple Box2D worlds simultaneously, then they should probably not use a
  task system. Otherwise you're likely to get oversubscription.
- Any callbacks you hook up to Box2D must be thread-safe, such as memory allocators.
- All of the limitations for single world simulation still apply.

## SIMD

Box2D uses [SIMD](https://en.wikipedia.org/wiki/Single_instruction%2C_multiple_data) instructions to
improve performance. On x64 CPUs it will use AVX2 if possible and otherwise fall back to SSE2. This
decision is made at runtime using the [CPUID](https://en.wikipedia.org/wiki/CPUID) instruction. On
ARM64 and Apple Silicon Box2D will use Neon instructions. You can completely disable SIMD using the
preprocessor argument `BOX2D_DISABLE_SIMD`.

