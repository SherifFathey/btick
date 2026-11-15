^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package btick
^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2026-11-14)
------------------
* Optimize monitor snapshot publishing by avoiding unnecessary full tree updates.
* Publish incremental blackboard updates with tree transitions.
* Reduce snapshot message size by using compact JSON serialization.
* Track tree structure revisions to trigger snapshot updates when needed.
