# Contributing

RoboSim is a C++17 project built with CMake 3.24 or newer. Use GCC 9+, Clang 10+,
or Visual Studio 2022 with C++ desktop development tools. The first configure
downloads pinned yaml-cpp and nlohmann/json releases.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

The executable is `build/robosim` on Linux/macOS, `build/Release/robosim.exe` with
Visual Studio, or `build/robosim.exe` with single-configuration Windows generators.
Run `robosim verify scenarios --output results/verification` to inspect bundles.
Local results and build folders are ignored; update tracked `results/all` only
when intentionally refreshing examples.

Use focused changes and behavior tests. Keep noise seeds explicit and use a
tolerance for floating-point comparisons. Native tests use explicit checks that
remain active in Release builds. GCC/Clang Linux builds support runtime checks
with `-DROBOSIM_SANITIZERS=ON -DCMAKE_BUILD_TYPE=Debug`.

CI tests Linux and Windows and runs AddressSanitizer/UndefinedBehaviorSanitizer on
Linux. See [migration notes](docs/migration.md) for Python differences. The original
Python source remains in Git history.
