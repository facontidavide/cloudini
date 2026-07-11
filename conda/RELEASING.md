# Releasing Cloudini as a conda package (pixi / prefix.dev + conda-forge)

`conda/recipe.yaml` builds a conda package that ships:

- `libcloudini_lib.so` (shared) + headers + a CMake package config
  (`find_package(cloudini_lib)` → `cloudini::cloudini_lib`)
- the `cloudini_rosbag_converter` CLI

The build is **hermetic**: it links conda's `zstd` / `lz4-c`, uses conda's
`cxxopts`, and consumes `mcap` from a pre-fetched source (no network during the
build script). The same recipe is used for both the prefix.dev channel and the
conda-forge submission.

## Prerequisites (one time)

```bash
pixi global install rattler-build        # already installed here
rattler-build auth login https://prefix.dev --token <YOUR_PREFIX_DEV_TOKEN>
# token: prefix.dev → Settings → API tokens (needs write to the 'cloudini' channel)
```

## Cut a release

1. **Pick the version** and make it consistent in three places (they must equal
   the git tag):
   - `cloudini_lib/CMakeLists.txt` → `project(cloudini_lib VERSION X.Y.Z)`
   - `conda/recipe.yaml` → `context.version`
   - (optional) `cloudini_lib/package.xml` → `<version>` (ROS manifest)

2. **Commit, tag, push:**
   ```bash
   git commit -am "release: X.Y.Z"
   git tag X.Y.Z
   git push origin main --tags
   ```

3. **Fill the source hash** in `conda/recipe.yaml`:
   ```bash
   curl -sL https://github.com/facontidavide/cloudini/archive/refs/tags/X.Y.Z.tar.gz \
     | sha256sum
   # paste into source[0].sha256
   ```

## Publish to your prefix.dev channel

```bash
# Build + upload + index in one step:
rattler-build publish ./conda/recipe.yaml --to https://prefix.dev/cloudini

# …or in two steps:
rattler-build build   --recipe conda/recipe.yaml -c conda-forge
rattler-build upload prefix -c cloudini ./output/**/*.conda
```

Verify from a clean env:

```bash
pixi init /tmp/cloudini-check && cd /tmp/cloudini-check
pixi project channel add https://prefix.dev/cloudini
pixi project channel add conda-forge
pixi add cloudini
pixi run cloudini_rosbag_converter --help
```

## Submit to conda-forge (broad reach: `pixi add cloudini` with no extra channel)

1. Fork `conda-forge/staged-recipes`.
2. Copy `conda/recipe.yaml` to `recipes/cloudini/recipe.yaml` in that fork
   (it is already conda-forge-compatible: pinned tarball + sha256,
   `extra.recipe-maintainers`, hermetic build, run_exports-driven run deps).
3. Open a PR. conda-forge CI builds linux-64/osx-64/osx-arm64. Once merged, a
   `cloudini-feedstock` repo is created for you to maintain; a bot opens version
   bump PRs automatically thereafter.

Notes for the conda-forge review:
- Windows is not enabled (POSIX-oriented CLI + `-msse4.1`); add a `bld.bat` +
  `skip` logic later if desired.
- PCL is intentionally not a dependency, so `pcl_conversion.hpp` ships but is
  only usable by consumers that bring their own PCL.

## Local dry-run without a tag

Build straight from a checkout (no tag/sha256 needed) using a `path:` source —
see the throwaway recipe used during bring-up. Handy for testing recipe/CMake
changes before cutting a tag.
```

## What the packaging touched in the library

To make the library installable/consumable outside ament, this release added to
`cloudini_lib/CMakeLists.txt` and its cmake modules:
- `install(EXPORT ...)` + generated `cloudini_libConfig.cmake` for non-ament builds
- `$ORIGIN/../lib` install RPATH so installed executables find the co-installed lib
- shared-variant zstd/lz4 selection when the library is shared
- `find_or_download_zstd.cmake`: accept conda's `zstd::libzstd_shared` / `zstd::libzstd`
- `find_or_download_mcap.cmake`: `-DMCAP_INCLUDE_DIR=` offline hook
- `mcap_converter` pinned `STATIC` so the CLI stays self-contained under `BUILD_SHARED_LIBS=ON`
