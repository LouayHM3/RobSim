# Reproduce test results

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

CTest writes logs under `build/Testing/Temporary` and scenario bundles under
`build/verification`. Run the CLI directly on Visual Studio builds with:

```powershell
.\build\Release\robosim.exe verify scenarios --output results/verification
```

Linux/macOS uses `./build/robosim`; single-configuration Windows builds use
`.\build\robosim.exe`. The [example summary](../results/all/summary.md) and reports
are generated data, not a substitute for testing your current build.
