# EVS 2ms Logger Work Folder

Use this folder as the **only** runtime location for `evs_2ms_logger`.

## Quick start

1. Configure the build (Release).
   ```bat
   cd C:\Project
   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
   ```
2. Build the EVS 2ms logger target (this also builds the HAL plugins).
   ```bat
   cmake --build build --config Release --target evs_2ms_logger
   ```
3. Double-click or run `run_evs_2ms_logger.bat` from this folder.

The batch script will:
- Run from `C:\Project\work_evs_2ms_logger`.
- Strip `C:\Program Files\Prophesee\...` entries from `PATH` for the session.
- Force HAL plugin paths to `C:\Project\build\lib\metavision\hal\plugins`.
- Force plugin search mode to `PLUGIN_PATH_ONLY` so Program Files plugins are ignored.
- Ensure the build `bin\Release` folder is first in `PATH` so DLL loading favors build artifacts.
- Launch `C:\Project\build\bin\Release\evs_2ms_logger.exe` using the build DLLs.

## What gets copied where

After the build:
- HAL plugins are built into `C:\Project\build\lib\metavision\hal\plugins`.
- The `evs_2ms_logger` post-build step copies all `*.dll` in that plugins folder to
  `C:\Project\build\bin\Release`, ensuring the plugin DLLs are in the executable directory.

If you rebuild the plugins, re-run the build step in this README to refresh the copied DLLs.

## Output location

All output stays under this work folder:

```
C:\Project\work_evs_2ms_logger\output\run_YYYYMMDD_HHMMSS\...
```

Use the `logs` folder if you want to redirect stdout/stderr into files.
