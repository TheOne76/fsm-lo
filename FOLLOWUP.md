# Known gaps and deferred work

Recorded during the ROS 2 port. None of these blocks the package; all were left
alone deliberately, and the reason is given so that the decision can be
revisited rather than rediscovered.

## Infinite and not-a-number ranges are not handled

A `LaserScan` may report a ray that struck nothing as infinity, or an invalid
reading as not-a-number. This package handles neither. Only a range of exactly
zero is recognised as invalid and filled in; anything else is fed to the
matcher as though it were a real distance.

An infinity entering the frequency transform contaminates every coefficient, so
a single such ray ruins the match for that scan pair.

This was not fixed during the port because doing so changes behaviour, and the
port's purpose was to demonstrate that behaviour had not changed. It should be
fixed next, together with a decision about what an infinite range means: treat
it as invalid and fill it in, or clamp it to `range_max`.

## The transform library plans a transform on every call

Three of the transform utilities create an FFTW plan, use it once, and destroy
it, on every call. Plans are expensive to create, and under the default
`FFTW_MEASURE` the creation runs timing trials.

Two consequences: execution time is far higher than it needs to be, and plan
choice can vary between calls, which is why the comparison build switches to
`FFTW_ESTIMATE`.

The node already caches the two plans it uses directly. The remaining three
should be cached the same way. Left alone during the port because caching them
changes which plan is used, and therefore the numbers.

## The default orientation bound differs between code and configuration

`t_bound` defaults to π/4, that is 0.785398, in the code, and to 0.786 in
`config/params.yaml`. The difference is small but neither value is obviously
intended, and a user reading one will not get the other.

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

## Deeper modernisation of the core was not attempted

The core builds clean as C++23 and its warnings are gone, but it is still
written in the style of 2016: triples of doubles rather than a named pose type,
output parameters rather than return values, raw owning pointers around the
transform buffers, and classes of static functions where namespaces would do.

The groundwork for changing that is in place. `test/test_core_golden.cpp`
compares the core against numbers captured from the reference build using
nothing but numeric fixtures, so it can gate every step of such a rewrite
without ROS or containers.
