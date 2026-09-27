# GPU (PowerVR SGX543MP4+) и Live Area — план второго этапа

## Что есть в материалах

В `Sony_Vita_Sdk_0945-YLoD\InstallFiles\[26]\sdk`:

| Файл | Что даёт |
|---|---|
| `documentation\en\pdf\SDK_doc\Graphics\GPU-Users_Guide_e.pdf` | архитектура GPU, режимы, тайлы, ограничения |
| `documentation\...\Graphics\libgxm-Overview_e.pdf`, `libgxm-Reference_e.pdf` | API GXM: очереди команд, шейдеры, состояния |
| `documentation\...\Graphics\libgxt-Overview_e.pdf`, `-Reference_e.pdf` | форматы текстур (GXT) |
| `documentation\...\Graphics\Shader_Compiler-Users_Guide_e.pdf` | компилятор шейдеров, форматы входов/выходов |
| `documentation\...\Graphics\Display-Overview_e.pdf`, `Display-Reference_e.pdf` | дисплей: буферы, регистры, синхронизация |
| `target\include\gxm\shader_patcher.h`, `target\include\gxt.h` | **бинарный формат шейдеров и текстур** |
| `target\include\display.h` | интерфейс дисплея |
| `host_tools\graphics\include\gxt\gxt_conversion.h`, `psp2gxt.exe` | конвертация GXT |

Плюс прошивочные модули, которые и надо эмулировать:
`Out\fs_dec\os0\us\libgxm_es4.elf` (пользовательский драйвер GXM),
`libgpu_es4.elf` (~139 КБ, GPU-драйвер), `Out\fs_dec\os0\kd\display.elf`,
`gpucoredump_es4.skprx`, `vipimg.skprx`.

## План

1. **Реверс `display.elf`**: регистры контроллера дисплея (уже есть каркас
   `KermitBlock::framebuffer()` в `src/hw/soc/display.cpp` — привести к реальным
   адресам и битовым полям из модуля). Цель — получить настоящий кадр в
   буфере, который читает SDL3-фронтенд (вкладка **Panel**, `F7`).
2. **Реверс `libgpu_es4.elf`**: это ядро GPU-драйвера: окно регистров SGX
   (base + смещения), управление очередями, прерывания, MMU GPU.
   Строим `src/hw/gpu` с регистровым интерфейсом и очередью команд.
3. **Реверс `libgxm_es4.elf`**: пользовательский уровень GXM формирует
   командные буферы. Нужен разбор формата команд (заголовки, состояния,
   draw calls, ссылки на шейдеры/текстуры). `shader_patcher.h` даёт структуру
   заголовка шейдера, `libgxm-Reference_e.pdf` — семантику состояний.
4. **Трансляция на OpenGL 4.x/GLSL**:
   * шейдеры GXM — это USSE-подобный ISA (SGX); нужен дизассемблер шейдеров
     (`Shader_Compiler-Users_Guide_e.pdf` + `shader_patcher.h`) и транслятор
     в GLSL; для первого кадра достаточно покрыть вершинный и пиксельный
     шейдеры Live Area;
   * тайловый рендерер SGX эмулируем как immediate-mode GL: набор состояний +
     отрисовка треугольников, render-to-texture через FBO;
   * текстуры GXT/компрессированные форматы распаковываем на CPU.
5. **Live Area**: после того как ядро поднимет `shell.self` и модули `vs0`,
   в дисплей попадёт кадр. Проверка — вкладка **Panel** и
   `--screenshot ... --screenshot-tab panel`.

## Инфраструктура, которая для этого уже есть

* SDL3-фронтенд с вкладкой Panel и оффскрин-скриншотами
  (`zeliboba_ui --screenshot out.bmp --screenshot-tab panel`);
* `KermitBlock::framebuffer(w, h, stride)` — точка подключения дисплея
  (поддерживаются RGB565 и RGBA8888);
* общая инфраструктура устройств: любое новое окно регистров сразу видно в
  отладчике (`devices`, `devget`/`devset`, `trace`) без дополнительного кода;
* `tools/zdis.cpp` для дизассемблирования модулей при реверсе.

## Порядок работ

GPU имеет смысл начинать только после того, как ядро (`os0`) реально
загружается и запускает модули: иначе нечего отрисовывать. Поэтому текущий
порядок — сначала `docs/BOOT.md` до стадии `kernel-running`, затем Venezia,
затем GPU.
