# Known Limitations {#known-limitations}

Box2D makes trade-offs for performance and stability. The behaviors on this page are known and
intentional. They are not bugs, so please don't report them as issues. Each section describes the
symptom, why it happens, and what to do about it.

## Contact recycling drifts on slowly rotating platforms

Dynamic bodies stacked on a large rotating platform can drift. One example is crates on the deck
of a ship. You can prevent this by disabling recycling on the platform.

```c
platformBodyDef.enableContactRecycling = false;
```

See also: [Contact recycling](simulation.md#contact-recycling).
