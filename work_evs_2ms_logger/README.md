# EVS 2ms Logger Work Folder

Use this folder as the **only** runtime location for `evs_2ms_logger`.

## Quick start

1. Build the project (Release).
2. Double-click or run `run_evs_2ms_logger.bat` from this folder.

The batch script will:
- Run from `C:\Project\work_evs_2ms_logger`.
- Strip `C:\Program Files\Prophesee\...` entries from `PATH` for the session.
- Force HAL plugin paths to `C:\Project\build\lib\metavision\hal\plugins`.
- Launch `C:\Project\build\bin\Release\evs_2ms_logger.exe` using the build DLLs.

## Output location

All output stays under this work folder:

```
C:\Project\work_evs_2ms_logger\output\run_YYYYMMDD_HHMMSS\...
```

Use the `logs` folder if you want to redirect stdout/stderr into files.
