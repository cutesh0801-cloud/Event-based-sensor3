# EVS 2ms Logger Work Folder

Use this folder as the **only** runtime location for `evs_2ms_logger`.

## Quick start (Release build with Visual Studio generator)

1. Configure the build (Release, Visual Studio generator).
   ```bat
   cd C:\Project
   cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release
   ```
2. Build the EVS 2ms logger target (this also builds the HAL plugins).
   ```bat
   cmake --build build --config Release --target evs_2ms_logger
   ```
3. **Always run from this folder**: double-click or run `run_evs_2ms_logger.bat`.

The batch script will:
- Run from `C:\Project\work_evs_2ms_logger`.
- Strip `C:\Program Files\Prophesee\...` entries from `PATH` for the session.
- Force HAL plugin paths to `C:\Project\build\lib\metavision\hal\plugins`.
- Force plugin search mode to `PLUGIN_PATH_ONLY` so Program Files plugins are ignored.
- Ensure the build `bin\Release` folder is first in `PATH` so DLL loading favors build artifacts.
- Launch `C:\Project\build\bin\Release\evs_2ms_logger.exe` using the build DLLs.

## 실행 방법 및 절차 (Program run steps)

1. 위 **Quick start**대로 Release 빌드를 완료합니다.
2. `C:\Project\work_evs_2ms_logger` 폴더로 이동합니다.
3. **반드시 `run_evs_2ms_logger.bat`로 실행**합니다. (직접 exe 실행 금지)
4. 프로그램 실행 후:
   - `o` 를 눌러 Camera ON.
   - 카메라 ON 이후에만 bias 제어가 가능합니다.
   - `b`/`B`/`n`/`+`/`-`/`[`/`]`/`p` 명령으로 bias 조작.
5. 종료는 `q`.

## Bias 조작 키 (새 UX)

Bias 조작은 **Camera ON 이후**에만 동작하며, 카메라 플러그인이 I_LL_Biases facility를 제공해야 합니다.
facility가 없으면 다음 메시지가 출력되고 bias 명령은 무시됩니다:
`이 디바이스는 bias 조절 미지원 (I_LL_Biases facility unavailable).`

- `b` : 사용 가능한 bias 목록과 현재 값을 출력 (name=value).
- `B` : `b` + bias info (가능한 경우 range/description/category/modifiable).
- `n` : bias 이름 선택 모드 (이름 입력 후 Enter). 예: `bias_diff_on`
- `+` : 선택 bias 값을 +step
- `-` : 선택 bias 값을 -step
- `]` : step 증가 (예: 1 -> 5 -> 10 -> 20 -> 50)
- `[` : step 감소
- `p` : 현재 선택 bias 이름/값/step 출력
- `o/f/s/e/q` : 기존 기능 유지

> Tip: `b`를 먼저 실행하면, 선택 bias가 비어 있을 때 첫 번째 bias가 자동 선택됩니다.

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
