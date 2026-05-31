# Recording and Replay

Box2D can record a simulation to a file and replay it later, reproducing the original run
exactly. A recording is a log of every world-mutating API call from `b2CreateWorld` onward.
Replaying it re-runs the engine on those same calls, so the bodies follow the same paths and
end up in the same state.

The main use is for debugging. You can record a session then load it in the sample app and view
the Box2D state as the simulation progresses.

Because a recording starts at world creation, **you must enable recording before the world is
created**. There is no way to snapshot a world mid-session and record from there. This is the
standard contract for replay-based debugging tools, and the cost of keeping the format aligned
with the stable public API rather than the engine's internal layout. Recording is cheap when
enabled and free when off, so it is reasonable to leave on in development builds.

## Recording

Set `recordingPath` in your world definition before creating the world. Recording begins
immediately, so the file always starts at `b2CreateWorld`.

```c
b2WorldDef worldDef = b2DefaultWorldDef();
worldDef.recordingPath = "session.b2rec";
b2WorldId worldId = b2CreateWorld( &worldDef );

// ... create bodies, step the world as usual ...
```

`recordingPath` is the only way to start recording. There is intentionally no
`BeginRecording`, since starting partway through would require snapshotting the engine's
internal state, which the design deliberately avoids.

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

b2RecPlayer_Restart( player );   // rewind to the first step, recreating the world
b2RecPlayer_Destroy( player );
```

`b2RecPlayer_Create` returns `NULL` if the file is missing or fails validation (see the
determinism contract below). `b2RecPlayer_IsAtEnd` reports when the recording is exhausted,
and `b2RecPlayer_HasDiverged` reports whether a recorded state hash failed to reproduce.
Divergence is non-fatal during playback so the viewer can keep playing and show where the
run starts to differ.

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
- **Recording must be enabled before the bug.** There is no mid-session start and no snapshot;
  a recording always begins at `b2CreateWorld`.
- **Friction and restitution callbacks must be reinstalled.** These mixers are pure functions
  of their inputs, so their results are not recorded. To replay a session that used a custom
  mixer, install the same mixer on the replay host. The defaults need nothing.
- **preSolve and customFilter callbacks are not supported while recording.** They carry user
  context and can change the simulation, so recording them faithfully is deferred. Installing
  one on a recording world fails loud rather than writing a file that would diverge on replay.
