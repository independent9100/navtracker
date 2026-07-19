# find_package(navtracker) consumer example

A minimal standalone project that consumes navtracker as an **installed package**
via `find_package(navtracker)` and links the exported `navtracker::navtracker_core`
target. It is the W6.4 install/export smoke test (see `docs/integration-guide.md`
§1, "How to consume it from your CMake build").

This is a *standalone* consumer — it is **not** built as part of navtracker's own
build. Build it after installing navtracker.

## Build

```bash
# 1. Configure + build navtracker, then install it to a prefix.
#    (from the navtracker repo root; use your Conan toolchain as usual)
conan install . -of=build -s build_type=Release -s compiler.cppstd=17 --build=missing
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build --target navtracker_core navtracker_nmea navtracker_t2t
cmake --install build --prefix /tmp/navtracker-install

# 2. Build this consumer against the installed package.
cmake -S examples/consumer_find_package -B /tmp/nt_consumer_build \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/build/conan_toolchain.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -Dnavtracker_DIR=/tmp/navtracker-install/lib/cmake/navtracker
cmake --build /tmp/nt_consumer_build
/tmp/nt_consumer_build/smoke
# -> navtracker find_package smoke OK: north=1112.96 m
```

## Dependency note

The exported consumer targets (`core` / `nmea` / `t2t`) have exactly one public
external dependency — **Eigen3** — so `find_package(navtracker)` re-finds only
Eigen3. Your build must be able to resolve it (the Conan toolchain above provides
it; a system Eigen3 or a `CMAKE_PREFIX_PATH` entry works too).
