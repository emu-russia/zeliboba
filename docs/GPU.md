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

## Состояние дисплейной части (раунд 167)

Проверено фактическим прогоном:

```
zeliboba_ui --screenshot C:\Work\PSVita\_scratch\panel.bmp --screenshot-tab panel --run 600000 -q
=>  screenshot written: ...panel.bmp (1280x720)
```

Вкладка **Panel** рисуется (скриншот 1280×720, проверен визуально) и честно
сообщает состояние:

> «no framebuffer yet — the kernel has not reached the display driver, so the
> Kermit display controller (0xE2100000, 960x544 panel) has not been programmed.
> The panel appears here as soon as the kernel publishes a buffer; RGB565 and
> RGBA8888 are both supported.»

То есть путь «контроллер дисплея → кадр → Panel» в модели **уже работает** и
ждёт только программирования регистров со стороны ядра; подпись внизу панели —
`panel: emulated display output; RGB565/RGBA8888 detected from stride`. Скриншот
этого состояния: `_scratch/panel.png`.

Карта регистров в `src/hw/soc/display.cpp` помечена как **ASSUMPTION**
(`display.elf`/`oled.elf` её не дали). Что дал разбор прошивки в этом раунде:

* `display.elf` (`fs_dec/os0/kd/display.elf`) **не обращается к MMIO вообще** —
  ни одного литерала в `0xE2000000-0xE3000000` и ни одной `movw/movt`-пары с
  таким адресом. Модули `os0` вообще работают с железом через сервисы ядра:
  сканер, который считает базу «использованной» только если константу в
  ближайших инструкциях реально берут как базу для `ldr/str`, по всему
  `fs_dec/os0` нашёл лишь `wlanbt_robin_img_ax.elf` (`0xE0252000`,
  `0xE0580000`) и `safemode.elf` (`0xE2010000`, `0xE36D0000`);
* зато литерал **`0xE2100000` действительно есть в `safemode.elf`** (VA
  `0x8120E5F8`) — то есть «безопасный режим» программирует дисплей напрямую, и
  это лучший источник настоящей карты регистров. Прямой `ldr [pc,…]` на этот
  литерал сканер не нашёл, значит ссылка идёт через ARM-код или через таблицу
  дескрипторов — следующий шаг по дисплею: разобрать `safemode.elf` вокруг
  `0x8120E5F8` (в `tools/` уже есть `zdis`, а `_scratch/dump_base_users.py`
  печатает загрузчики литералов) и уточнить `display.cpp`.

Инструменты этого раунда: `ZLB_ARM_COV=1` + `cov`/`cov save` (карта покрытия ARM
по 2 байта на бит) и `_scratch/scan_missed.py` (пропущенные рёбра потока
управления) — ими проверено, что внутри Thumb-кода NSKBL **нет** пропущенных
вызовов и прогон детерминирован.
