# Recording and Replay

Box2D can record a simulation to a file and replay it later, reproducing the original run
exactly. A recording is a log of every world-mutating API call from `b2CreateWorld` onward.
Replaying it re-runs the engine on those same calls, so the bodies follow the same paths and
end up in the same state.

The main use is for debugging. You can record a session then load it in the sample app and view
the Box2D state as the simulation progresses.

The simplest recording starts at world creation: enable it before the world is created and the
file logs every call from `b2CreateWorld` on. Recording is cheap when enabled and free when off,
so it is reasonable to leave on in development builds.

A recording can also start mid-session. `b2World_StartRecording` serializes a snapshot of the
live world into the file, which then continues with the same call log. This serves the "a bug
appeared after 30 seconds and only then did I turn recording on" case that a from-creation
recording cannot. See [Snapshots and mid-stream recording](#snapshots-and-mid-stream-recording).

## Recording

Set `recordingPath` in your world definition before creating the world. Recording begins
immediately, so the file always starts at `b2CreateWorld`.

```c
b2WorldDef worldDef = b2DefaultWorldDef();
worldDef.recordingPath = "session.b2rec";
b2WorldId worldId = b2CreateWorld( &worldDef );

// ... create bodies, step the world as usual ...
```

`recordingPath` records from `b2CreateWorld`. To start partway through a session instead, use
`b2World_StartRecording` (below), which snapshots the live world into the file first.

While a world is recording you can copy the file out at any time without interrupting capture:

```c
// Snapshot the recording so far; the world keeps recording afterwards.
b2World_SaveRecording( worldId, "my_report.b2rec" );
```

To end capture cleanly, stop it explicitly or just destroy the world.

```c
b2World_StopRecording( worldId );   // flushes and closes the file
// b2DestroyWorld also flushes and closes any active recording.
```

## Replay

The simplest way to check a recording is the headless validator. It re-runs the engine over
the file and confirms every recorded id and per-step state reproduces.

```c
bool ok = b2ValidateReplayFile( "session.b2rec", 0 );
// ok == false means replay diverged from the recording.
```

`workerCount` selects how many worker threads the replay world uses. Pass `0` to use the
count stored in the file. Box2D is deterministic across thread counts, so a session recorded
single-threaded replays identically with four workers and vice versa.

For stepping through a recording frame by frame, for example to drive a viewer or inspect the
world between steps, use the player handle:

```c
b2RecPlayer* player = b2RecPlayer_Create( "session.b2rec", 0 );
b2WorldId worldId = b2RecPlayer_GetWorldId( player );

while ( b2RecPlayer_StepFrame( player ) )
{
    // The replay world now holds the state after b2RecPlayer_GetFrame( player ) steps.
    // Read it with the normal b2Body_Get* and b2World_* functions, or draw it.
}

b2RecPlayer_Restart( player );   // rewind to frame 0 in place; the world id stays the same
b2RecPlayer_Destroy( player );
```

`b2RecPlayer_Create` returns `NULL` if the file is missing or fails validation (see the
determinism contract below). `b2RecPlayer_IsAtEnd` reports when the recording is exhausted,
and `b2RecPlayer_HasDiverged` reports whether a recorded state hash failed to reproduce.
Divergence is non-fatal during playback so the viewer can keep playing and show where the
run starts to differ.

## Snapshots and mid-stream recording {#snapshots-and-mid-stream-recording}

A snapshot is a serialized image of a world's simulation state at a step boundary. It captures
everything the engine needs to continue the simulation: bodies, shapes, joints, contacts with
their warm-start impulses, the island and sleep partition, the broad-phase trees, and the id
pools. It does not capture host wiring (worker count, task callbacks, user data, the friction
and restitution mixers); that is rebuilt or reinstalled at restore.

### Recording from the current state

`b2World_StartRecording` begins a recording from a live world instead of from `b2CreateWorld`.
It writes a snapshot of the world into the file, then logs subsequent calls as usual. Call it
at a step boundary.

```c
// Run normally, no recording yet...
for ( int i = 0; i < 1800; ++i )
    b2World_Step( worldId, 1.0f / 60.0f, 4 );

// Something looks wrong. Start recording from here.
b2World_StartRecording( worldId, "from_the_bug.b2rec" );

for ( int i = 0; i < 600; ++i )
    b2World_Step( worldId, 1.0f / 60.0f, 4 );

b2World_StopRecording( worldId );
```

The resulting file replays exactly like a from-creation file with `b2ValidateReplayFile`, the
player, or the viewer. The only difference is that it opens by deserializing the snapshot rather
than by replaying a `b2CreateWorld` call. `b2World_SaveRecording` and `b2World_StopRecording`
work the same for both kinds.

### Saving and restoring a world directly

The same machinery is available without a file, as a public API for save states and rollback:

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

### Layout gate

Unlike the build-hash, which is informational for from-creation files, a snapshot is a raw
struct image and the reader's build must have **identical struct layouts**. The image carries a
layout hash, and restore refuses an image whose hash differs. A rejected image (bad magic,
version, or layout) leaves the target world unchanged. A snapshot file therefore cannot replay
across a build whose layout changed, which is stricter than the cross-build tolerance of a
from-creation file.

## Viewing a recording

The samples app has a **Replay** category with a recording viewer:

- **Replay File** loads a `.b2rec` and plays it back using a `b2RecPlayer`. It supports
  play, pause, single-step, restart, and camera control. A `DIVERGED` overlay appears if a
  recorded state hash fails to reproduce, which is a real determinism break, not a viewer bug.

## Determinism contract

Replay reproduces the original run exactly only when the replaying build matches the recording
build in the ways that affect the math. Some of this the file header enforces on load, and the
rest is your responsibility:

- **Pointer width** and **endianness** are enforced. The format stores some definitions as raw
  bytes, so the architecture must match; `b2RecPlayer_Create` and `b2ValidateReplayFile` reject
  a file that disagrees rather than producing a silently wrong replay. The format version is
  enforced the same way.
- **Floating-point environment** must match. Box2D builds with `-ffp-contract=off` so fused
  multiply-add does not change results. Building with `-ffast-math` is unsupported.

## Build-hash

The header also records the engine's git build hash, but this is informational, not
an enforced gate. A recording from a different commit still opens and plays. The hash just lets
a tool warn that the builds differ.

```c
uint32_t engineHash = b2GetBuildHash();                 // the running engine
uint32_t fileHash   = b2RecPlayer_GetBuildHash( player ); // the recording
```

The hash is stamped at CMake configure time, so it can lag by a commit
until the next configure. This only makes the warning coarse, never blocks replay. The Replay
sample shows the recorded hash on load and draws a `build mismatch` line when the two differ
and neither is zero.

## Spatial queries

Overlap and cast queries issued during a recorded step (ray casts, shape casts, overlap
tests, and the character mover casts) are recorded too. On replay each query is re-issued
against the replayed world and its results are compared against what was recorded, so a query
that returns different hits is flagged like any other divergence. `b2RecPlayer_DrawFrameQueries`
draws the queries from the most recently replayed frame, layered on top of the world; call it
after `b2World_Draw`.

## Limitations

- **User data is not preserved.** `userData` pointers are host addresses with no meaning in
  the replay process, so they are written as zero. Code that keys off user data during replay
  will not see the original pointers.
- **Snapshots require a matching struct layout.** A from-creation file replays across builds as
  long as the float environment and architecture match, but a snapshot (and a file made by
  `b2World_StartRecording`) is a raw struct image gated on an exact layout hash, so it will not
  load into a build whose internal layout changed.
- **Friction and restitution callbacks must be reinstalled.** These mixers are pure functions
  of their inputs, so their results are not recorded. To replay a session that used a custom
  mixer, install the same mixer on the replay host. The defaults need nothing.
- **preSolve and customFilter callbacks are not supported while recording.** They carry user
  context and can change the simulation, so recording them faithfully is deferred. Installing
  one on a recording world fails loud rather than writing a file that would diverge on replay.
