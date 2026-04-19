# InSitu Cache performance model — see prompt/insitu_cache_architecture.md
#
# The three components (controller, coalescer, interco) use the modern `add_sources()`
# pattern (same as cache_v3), which compiles the .cpp into a model named after the
# Python module path. No vp_model() entries are needed here.
#
# Python-only file (insitu_cache_tile.py) is picked up automatically by the
# GVSOC_MODULES mechanism because this directory is under core/models which is in
# GVSOC_MODULES (see top-level Makefile).
