# Recording and Replay

Box2D can record a simulation into a memory buffer and replay it later, reproducing the original
run exactly. A recording is a snapshot of the world at the moment recording starts followed by a
log of every world-mutating API call after that. Replaying it re-runs the engine from the
snapshot over those same calls, so the bodies follow the same paths and end up in the same state.

The main use is for debugging. You can record a session, save the buffer to a file, then load it
in the sample app and view the Box2D state as the simulation progresses.

You own the recording buffer. Box2D records into it and grows it as needed; you save it to disk
and free it. The library does no file I/O of its own for recording, beyond two optional
convenience helpers (`b2SaveRecordingToFile` / `b2LoadRecordingFromFile`).

A recording can start before the first step, capturing the whole session, or mid-session for the
"a bug appeared after 30 seconds and only then did I turn recording on" case. Either way the
buffer opens with a snapshot of the world as it stands when recording starts, so there is one
code path for both.

## Recording

Create a recording buffer, start recording the world into it, then run the simulation as usual.
Start before the first step to capture everything.

```c
b2WorldId worldId = b2CreateWorld( &worldDef );

b2Recording* recording = b2CreateRecording( 0 );   // 0 = small default capacity
b2World_StartRecording( worldId, recording );       // snapshots the world, then logs calls

// ... create bodies, step the world as usual ...

b2World_StopRecording( worldId );
```

`b2CreateRecording` takes a byte capacity to pre-size the buffer. The buffer still grows on
demand, so any value is safe; pre-sizing just avoids reallocations during a long session. Pass
`0` for a small default.

`b2World_StartRecording` must be called at a step boundary. It serializes a snapshot of the
current world as the seed, so it works whether you call it before any bodies exist or deep into
a running simulation. It has no effect if the world is already recording.

When you are done, save the buffer and free it. File I/O is yours to do; the convenience helper
writes the raw bytes:

```c
b2SaveRecordingToFile( recording, "session.b2rec" );   // or fwrite the bytes yourself:
// const uint8_t* data = b2Recording_GetData( recording );
// int size = b2Recording_GetSize( recording );

b2DestroyRecording( recording );
```

Stopping is optional: `b2DestroyWorld` detaches an active recording for you. The recording buffer
outlives the world, so you can still save it after the world is gone. You can also keep
simulating after `b2World_StopRecording` without recording, and reuse the same handle for a fresh
recording with another `b2World_StartRecording`.

## Replay

The simplest way to check a recording is the headless validator. It re-runs the engine over the
recorded bytes and confirms every recorded id and per-step state reproduces.

```c
const uint8_t* data = b2Recording_GetData( recording );
int size = b2Recording_GetSize( recording );

bool ok = b2ValidateReplay( data, size, 0 );
// ok == false means replay diverged from the recording.
```

`workerCount` selects how many worker threads the replay world uses. Pass `0` for the serial
single-worker fallback. Box2D is deterministic across thread counts, so a session recorded
single-threaded replays identically with four workers and vice versa.

To replay a recording from disk, load it into a buffer first:

```c
b2Recording* loaded = b2LoadRecordingFromFile( "session.b2rec" );
bool ok = b2ValidateReplay( b2Recording_GetData( loaded ), b2Recording_GetSize( loaded ), 0 );
b2DestroyRecording( loaded );
```

For stepping through a recording frame by frame, for example to drive a viewer or inspect the
world between steps, use the player handle. The player copies the bytes it is given, so you can
free the source buffer immediately after creating it.

```c
b2RecPlayer* player = b2RecPlayer_Create( data, size, 0 );
b2WorldId worldId = b2RecPlayer_GetWorldId( player );

while ( b2RecPlayer_StepFrame( player ) )
{
    // The replay world now holds the state after b2RecPlayer_GetFrame( player ) steps.
    // Read it with the normal b2Body_Get* and b2World_* functions, or draw it.
}

b2RecPlayer_Restart( player );   // rewind to frame 0 in place; the world id stays the same
b2RecPlayer_Destroy( player );
```

`b2RecPlayer_Create` returns `NULL` if the bytes are malformed or fail the layout gate (see the
determinism contract below). `b2RecPlayer_IsAtEnd` reports when the recording is exhausted, and
`b2RecPlayer_HasDiverged` reports whether a recorded state hash failed to reproduce. Divergence
is non-fatal during playback so the viewer can keep playing and show where the run starts to
differ.

## Snapshots

A recording is seeded by a snapshot: a serialized image of a world's simulation state at a step
boundary. It captures everything the engine needs to continue the simulation: bodies, shapes,
joints, contacts with their warm-start impulses, the island and sleep partition, the broad-phase
trees, and the id pools. It does not capture host wiring (worker count, task callbacks, user
data, the friction and restitution mixers); that is rebuilt or reinstalled at restore.

The same machinery is available directly, without recording, as a public API for save states and
rollback:

```c
// Serialize: query the size, then fill a buffer you own.
int size = b2World_Snapshot( worldId, NULL, 0 );
uint8_t* image = malloc( size );
b2World_Snapshot( worldId, image, size );

// ... keep simulating ...

// Restore the same world in place. Ids you held at the snapshot instant keep working.
b2World_Restore( worldId, image, size );

// Or load the image into a brand new world. The new world has a fresh id, so origin ids
// do not match it, and its host wiring is reset to defaults.
b2WorldId loaded = b2CreateWorldFromSnapshot( image, size, /*workerCount*/ 0 );
```

`b2World_Restore` keeps the world's slot and generation, so the `b2WorldId` and every
`b2BodyId` / `b2ShapeId` / `b2JointId` / `b2ChainId` you held at the snapshot instant resolve
again. Ids minted after the snapshot fail validation after a restore rather than aliasing a
different object. `b2CreateWorldFromSnapshot` allocates a new world, so use it only when there
is no existing world to restore into.

Like recording, snapshots are worker-count independent: a world snapshotted single-threaded
restores and continues with any worker count.

## Viewing a recording

The samples app has a **Replay** category with a recording viewer:

- **Replay File** loads a `.b2rec` into a buffer and plays it back using a `b2RecPlayer`. It
  supports play, pause, single-step, restart, and camera control. A `DIVERGED` overlay appears
  if a recorded state hash fails to reproduce, which is a real determinism break, not a viewer
  bug.

You can also record any sample: open the **Recording** controls in the diagnostics drawer, set a
file name, and press **Record**. The sample restarts with recording on, and **Stop** saves the
buffer to the file.

## Determinism contract

Replay reproduces the original run exactly only when the replaying build matches the recording
build in the ways that affect the math. Some of this the format enforces on load, and the rest is
your responsibility:

- **Struct layout** is enforced. A recording opens by deserializing a snapshot, which is a raw
  struct image, so the reader's build must have identical struct layouts. The image carries a
  layout hash and `b2RecPlayer_Create` / `b2ValidateReplay` reject a recording whose hash differs
  rather than producing a silently wrong replay. A recording therefore does not replay across a
  build whose internal layout changed.
- **Pointer width**, **endianness**, and the format version are enforced the same way.
- **Floating-point environment** must match. Box2D builds with `-ffp-contract=off` so fused
  multiply-add does not change results. Building with `-ffast-math` is unsupported.

## Build-hash

The header also records the engine's git build hash, but this is informational, not an enforced
gate. The hash lets a tool warn that the builds differ.

```c
uint32_t engineHash = b2GetBuildHash();                   // the running engine
uint32_t fileHash   = b2RecPlayer_GetBuildHash( player ); // the recording
```

The hash is stamped at CMake configure time, so it can lag by a commit until the next configure.
This only makes the warning coarse. The Replay sample shows the recorded hash on load and draws a
`build mismatch` line when the two differ and neither is zero.

## Spatial queries

Overlap and cast queries issued during a recorded step (ray casts, shape casts, overlap tests,
and the character mover casts) are recorded too. On replay each query is re-issued against the
replayed world and its results are compared against what was recorded, so a query that returns
different hits is flagged like any other divergence. `b2RecPlayer_DrawFrameQueries` draws the
queries from the most recently replayed frame, layered on top of the world; call it after
`b2World_Draw`.

## Limitations

- **Recordings require a matching struct layout.** A recording is seeded by a raw struct-image
  snapshot gated on an exact layout hash, so it will not load into a build whose internal layout
  changed, even if the float environment and architecture match.
- **User data is not preserved.** `userData` pointers are host addresses with no meaning in the
  replay process, so they are written as zero. Code that keys off user data during replay will
  not see the original pointers.
- **Friction and restitution callbacks must be reinstalled.** These mixers are pure functions of
  their inputs, so their results are not recorded. To replay a session that used a custom mixer,
  install the same mixer on the replay host. The defaults need nothing.
- **preSolve and customFilter callbacks are not supported while recording.** They carry user
  context and can change the simulation, so recording them faithfully is deferred. Installing one
  on a recording world fails loud rather than writing a recording that would diverge on replay.
