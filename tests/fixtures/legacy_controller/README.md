# Frozen legacy controller outputs

These policy arrays were captured before deleting the legacy C++ controller
(source tree at commit 204aaf8). They come from runtime/evidence/ocs2_binding/
whole_body/replay.mat and centroidal/replay.mat, variable `old`. Only that
variable is retained; tests must never regenerate it from the new binding.

Each has 2401 rows: evaluated state, input, mode and query time. Tests reuse
the existing frozen observation/reference fixtures. Whole-body has 95 columns;
centroidal has 72. Both were exactly equal to direct-binding replay before removal.

`sha256.json` records the committed MAT-file checksums. These are output-only
regression fixtures; replay inputs still come from runtime/evidence.
