# Vendored ikd-tree provenance

This directory contains the build-critical source files from `ikd-tree`:

- Upstream: https://github.com/alvgaona/ikd-tree
- Release tag: `v0.1.0`
- Commit: `808ec4d481cd99f95054796f46be152507dc849e`
- License: GPL-2.0-only (see `LICENSE`)

The source is vendored so the ROS 2 workspace builds reproducibly without a
Pixi environment, a precompiled architecture-specific archive, or network
access during the build. Only the library build files and source are included;
upstream examples, benchmarks, CI files, and package-manager metadata are
excluded.

Imported file SHA-256 values:

```text
3c2c7a0de578b03e74b033241fa0575ef7f9dc0b330279f9d9ddbe88d439fc23  CMakeLists.txt
8177f97513213526df2cf6184d8ff986c675afb514d4e68a404010521b880643  LICENSE
e66d0a489e72dce4cf0133eb32de3350cd3a64a7b98720adbae9e1d5086c3a6c  cmake/ikd_treeConfig.cmake.in
c162238b998eb77e18070ae51d8cd0df36761178c6372fa4557757d0e88136b4  include/ikd_tree/ikd_tree.h
425bfc639025b282183ee2562434a39cb8212f83582ca98a3d8ddc1f74177179  src/ikd_tree.cpp
```
