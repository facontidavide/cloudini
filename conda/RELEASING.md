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
pixi global install rattler-build
rattler-build auth login prefix.dev --oauth
```

For CI, prefer prefix.dev Repository Access / trusted publishing (OIDC), scoped
to the release repository and workflow, with `id-token: write` in GitHub Actions.
Pin the build-tool version in CI. See the official
[publishing guide](https://prefix.dev/docs/prefix/channels/publish-packages).

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
   git push origin main X.Y.Z
   ```

3. **Fill the source hash** in `conda/recipe.yaml`:
   ```bash
   curl -fL https://github.com/facontidavide/cloudini/archive/refs/tags/X.Y.Z.tar.gz \
     -o /tmp/cloudini-X.Y.Z.tar.gz
   sha256sum /tmp/cloudini-X.Y.Z.tar.gz
   # paste into source[0].sha256
   ```

   Commit the recipe hash separately; do not move the tag to include its own
   archive hash. Source changes after an existing tag require a new release tag
   (or an explicit recipe patch), otherwise the recipe still builds the old code.

## Publish to your prefix.dev channel

```bash
# Build and test before uploading the exact artifact:
rattler-build build   --recipe conda/recipe.yaml -c conda-forge
sha256sum output/linux-64/cloudini-*.conda
```

Before publishing, install the built artifact in a fresh environment and check
both the CLI and a CMake consumer. Rattler-Build runs the recipe's package tests
in an isolated environment; the standalone consumer check is also available at
`cloudini_lib/test/install`. Keep the recipe, source hash, build log, tool
versions, and artifact digest. Build and test each platform you intend to publish;
a passing Linux build does not validate macOS.

Then upload the tested artifact (substitute its exact filename):

```bash
rattler-build upload prefix -c cloudini output/linux-64/cloudini-X.Y.Z-BUILD.conda
```

For recipe-only corrections to an already published version, increase
`build.number`, rebuild, and publish a new filename. Do not overwrite an existing
artifact with `--force`; yank a broken build after publishing its replacement.

Verify the published version from a clean env:

```bash
pixi init /tmp/cloudini-check && cd /tmp/cloudini-check
pixi workspace channel add https://prefix.dev/cloudini
pixi workspace channel add conda-forge
pixi add 'cloudini==X.Y.Z'
pixi run cloudini_rosbag_converter --help
```

## Submit to conda-forge (broad reach: `pixi add cloudini` with no extra channel)

1. Fork `conda-forge/staged-recipes`.
2. Copy `conda/recipe.yaml` to `recipes/cloudini/recipe.yaml` in that fork
   including any recipe test files or patches. Run the staged-recipes checks;
   a local prefix.dev build alone does not establish conda-forge acceptance.
3. Open a PR and validate the selected platform builds. Once merged, a
   `cloudini-feedstock` repo is created for you to maintain; a bot opens version
   bump PRs automatically thereafter.

Notes for the conda-forge review:
- Windows is explicitly skipped in the recipe; enabling it requires a supported
  Windows build script and a passing package test.
- PCL is intentionally not a dependency, so `pcl_conversion.hpp` ships but is
  only usable by consumers that bring their own PCL.

## Local dry-run without a tag

Copy the recipe to a temporary directory. Replace only its first source entry
(`url` and `sha256`) with an absolute checkout path, leaving the pinned MCAP
source unchanged:

```yaml
source:
  - path: /absolute/path/to/cloudini-checkout
  # Keep the original MCAP source entry here.
```

Run `rattler-build build --recipe /path/to/temporary/recipe.yaml -c conda-forge`.
This tests untagged CMake changes; do not publish this temporary recipe. Before
publishing, build again from the release recipe's pinned archive.

## What the packaging touched in the library

To make the library installable/consumable outside ament, this release added to
`cloudini_lib/CMakeLists.txt` and its cmake modules:
- `install(EXPORT ...)` + generated `cloudini_libConfig.cmake` for non-ament builds
- `$ORIGIN/../lib` install RPATH so installed executables find the co-installed lib
- shared-variant zstd/lz4 selection when the library is shared
- `find_or_download_zstd.cmake`: accept conda's `zstd::libzstd_shared` / `zstd::libzstd`
- `find_or_download_mcap.cmake`: `-DMCAP_INCLUDE_DIR=` offline hook
- `mcap_converter` pinned `STATIC` so the CLI stays self-contained under `BUILD_SHARED_LIBS=ON`
