# msvc — решение для Visual Studio 2026

Здесь лежат проекты MSBuild, из которых собрано решение `zeliboba.slnx`
(и эквивалентный ему классический `zeliboba.sln` в корне репозитория). Оба
открываются в Visual Studio 2026 и собирают то же самое, что и CMake-сборка,
но без CMake: F5, отладчик, IntelliSense и точки останова работают сразу.

## Как открыть

1. `zeliboba.slnx` (в корне репозитория) — двойной щелчок или
   *File → Open → Project/Solution*.
2. Стартовый проект — **zeliboba_ui**, поэтому F5 сразу поднимает эмулятор.
   Консольный отладчик запускается через *Set as Startup Project* на `zeliboba`.
3. Рабочий каталог отладки выставлен в корень репозитория, так что
   `ZLB_WORKSPACE_DIR` и относительные пути к прошивке резолвятся как в
   `build.ps1`.

Из командной строки:

```powershell
msbuild zeliboba.slnx -p:Configuration=Release -p:Platform=x64 -m
msbuild zeliboba.sln  -p:Configuration=Debug   -p:Platform=x64 -m
```

## Из чего состоит решение

| Проект | Тип | Что собирает |
|---|---|---|
| `zeliboba_core` | статическая библиотека | `src/{common,bus,cpu,hw,loader,machine,debug}` |
| `zeliboba_ui` | приложение | SDL3-фронтенд, `src/ui` |
| `zeliboba` | приложение | консольный отладчик, `src/main.cpp` |
| `zlb_tests` | приложение | самотесты, `tests/*.cpp` |
| `emmc_rebuild`, `zdis`, `arm_probe` | приложения | инструменты из `tools/*.cpp` |

## Договорённости

* **Список исходников задан масками** (`src\cpu\**\*.cpp` и т. п.), поэтому
  новый `.cpp` в этих каталогах подхватывается сам — как и в `CMakeLists.txt`.
* Настройки компилятора повторяют `CMakeLists.txt`: C++20, `/utf-8 /permissive-
  /Zc:__cplusplus /W3 /MP /EHsc /fp:precise`, те же `ZLB_ROOT_DIR` и
  `ZLB_WORKSPACE_DIR`, `ZLB_HAVE_SDL3=1` только у UI.
* Вывод намеренно совпадает с CMake-сборкой: `build\bin\*.exe`, поэтому
  `verify.ps1` и команды из README работают после любого варианта сборки.
  Промежуточные файлы MSBuild лежат отдельно, в `build\msvc\`, и не мешают
  CMake-дереву.
* Общие свойства живут в `Directory.Build.props` **внутри `msvc/`**: так MSBuild
  применяет их только к этим проектам и не трогает `.vcxproj`, которые CMake
  генерирует в `build/`.
* `PlatformToolset` зафиксирован как `v145` (инструменты Visual Studio 2026).
* Пути для макросов передаются через **прямые** слэши: они попадают внутрь
  строковых литералов C++, где обратный слэш был бы escape-последовательностью.

CMake-сборка (`build.ps1`) остаётся основной; это решение — удобный способ
работать в IDE.
