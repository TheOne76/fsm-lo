# Known gaps and deferred work

Recorded during the ROS 2 port. None of these blocks the package; all were left
alone deliberately, and the reason is given so that the decision can be
revisited rather than rediscovered.

## An unmeasurable range is treated as no reading, but nothing is clamped

Infinity, not-a-number, zero and any negative range are all taken to mean the
sensor could not measure that ray, and are filled in from the rays either side.
A scan with nothing valid in it is refused.

What is still open is whether a ray that reports `range_max` exactly, which
some drivers use to mean "nothing within range" rather than "a surface at
exactly this distance", should be treated the same way. It currently is not.
Deciding that needs a survey of what drivers actually do.

## The core header is duplicated in the `fsm` repository

`include/fsm_lo/fsm_core.hpp` and `include/fsm.h` in
[`fsm`](https://github.com/li9i/fsm) are two copies of the same algorithm that
have drifted apart. Differences found while porting:

- `fsm` carries measurement noise parameters, a terminal constraint and an
  early gear-up feature that this copy does not.
- This copy carries the transform accumulation, scan subsampling and gap
  filling that the ROS wrapper needs, which `fsm` does not.
- One index variable that selects the best candidate has a different type in
  each copy, which can select a different candidate.

Only this copy received the eight corrections listed in the readme. The `fsm`
copy still carries all of them.

Reconciling the two is worth doing and is a job in itself. It was explicitly
out of scope for the port.

## The two formatting linters are switched off

`ament_cmake_uncrustify` and `ament_cmake_cpplint` impose a brace and wrapping
style this package has never used. Satisfying them means reformatting 3800
lines of inherited numerical code, which is the riskiest change available for
no behavioural gain. Every other check `ament_lint_common` provides is on.

## The recovery path is untested across versions

The matcher falls back on a randomly seeded search when a match goes badly.
`rng_seed` pins it so a run can be reproduced within one build, and there is a
unit test for that, but the cross version comparison deliberately never
exercises it: the synthetic motion is small enough that recovery never fires.

The reason is that the standard library does not specify how a distribution
turns random bits into a number, so two implementations may differ even from
the same seed. Comparing the recovery path across versions would first need
that conversion written out explicitly rather than taken from the library.

## The comparison harness does not assert that recovery stayed quiet

Because recovery cannot reproduce across builds, a comparison run in which it
fired would be meaningless, and the harness is supposed to fail rather than
report a difference it cannot explain. It does not check.

The node now reports the count truthfully and logs a warning whenever a match
needed recovery, so the evidence exists in the run output on the ROS 2 side; no
warning appears in any of the six scenarios. What is missing is the harness
reading it and failing on it, on both sides.

The reason it was left is cost against benefit. Closing it properly means
adding the same reporting to the ROS 1 reference build, which means rebuilding
the pinned container and regenerating every reference recording, disturbing a
proof that currently passes. The six scenarios agree to better than 4e-15,
which is itself strong evidence that the random path never fired on either
side, since it could not have agreed to that precision if it had.

## The recovery seed exists on the port branch only

`rng_seed` was meant to be threaded through on the correction branch as well,
so that both sides of the comparison could pin the recovery search. It reached
only the port branch. The correction branch still seeds from hardware entropy
with no way to pin it.

This changes nothing about the reference recordings, which were captured from a
build where recovery never fired, so the seed would have had nothing to
influence. Adding it now would invalidate a working reference for no gain. It
matters only if the recovery path is ever compared across versions, which the
entry above already says needs other work first.

## Deeper modernisation of the core was not attempted

The core builds clean as C++23 and its warnings are gone, but it is still
written in the style of 2016: triples of doubles rather than a named pose type,
output parameters rather than return values, raw owning pointers around the
transform buffers, and classes of static functions where namespaces would do.

The groundwork for changing that is in place. `test/test_core_golden.cpp`
compares the core against numbers captured from the reference build using
nothing but numeric fixtures, so it can gate every step of such a rewrite
without ROS or containers.
