# Phase 1 — Record & replay

**Goal:** Write both streams to disk with arrival timestamps, and replay them through the pipeline identically.

**Gate:** Replaying one recording twice produces bit-identical output.

**Status:** Not started — blocked on Phase 0.

Do this *before* any fusion work. Every later phase is a parameter you will tune
dozens of times, and without replay each tweak costs a re-walk of the room
instead of a re-run of a file. It looks skippable and is the single highest
-leverage thing in the plan.

Both clocks (ARKit device time and Mac arrival time) must be recorded so Phase 2
can be solved offline from data captured today.
