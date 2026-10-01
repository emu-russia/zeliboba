# zeliboba

![zeliboba](artwork/png/zeliboba-banner.png)

Низкоуровневый эмулятор PlayStation Vita на C++20 с бэкендом SDL3, отладчиком и
реконструированным образом eMMC. Цель первого этапа — довести загрузку прошивки
**1.04** до ядра (Kernel), а не до Live Area.

```
CMeP (F00D, MeP-c5)  -> first_loader -> second_loader -> secure_kernel
ARM Cortex-A9        -> kernel_boot_loader -> kernel (os0)
Ernie (RL78 syscon)  -> питание/сброс/RTC/SC-канал/eMMC-хост
```

Текущее состояние (подробности, адреса и метрики — `docs/STATUS.md`):

* CMeP first_loader проходит **всю** свою цепочку проверок (заголовок ENP, SHA-256,
  RSA через Bignum, digest-таблица, keyring) и рапортует `success` в mailbox;
* образ распаковывается **настоящим AES-128-CBC** ключом чипа (`ENC_KEY||ENC_IV`,
  слот keyring 10) прямо в `0x40000`, и вторая стадия реально исполняется оттуда
  (нужно ставить `second_loader.enc`, а не `.enp`);
* вторая стадия доходит до конца — до вызова сервиса `jmp 0x5FF00`, после чего
  CMeP перезапускается на secure kernel (это подстановка: кода сервиса в дампах
  нет, см. `docs/STATUS.md` §4);
* ARM `kernel_boot_loader` исполняет настоящий код, распаковывает NSKBL своим
  `sceArlzDecode` и передаёт управление в Non-Secure (`SCR.NS=1`);
* NSKBL доходит до чекпойнта `0xA9`: том `os0` читается и `psp2bootconfig.skprx`
  находится, но данные файла не читаются — устройство хранения, к которому
  обращается драйвер (VA `0x240`), в модели никем не заполняется.

Ближайший шаг — научить это устройство отвечать на управляющий запрос драйвера.

## Что внутри

| Компонент | Где | Состояние |
|---|---|---|
| Шина, RAM, MMIO, трейс | `src/bus/` | готово |
| Абстракция CPU и фабрика ядер | `src/cpu/cpu.h`, `src/cpu/factory.h` | готово |
| MeP-c5 (CMeP/F00D) | `src/cpu/mep/` | работает на реальном first_loader |
| ARM Cortex-A9 (ARMv7-A + Thumb-2 + VFP + MMU) | `src/cpu/arm/` | исполняет реальный KBL, VFP/NEON в работе |
| RL78 (Ernie) | `src/cpu/rl78/` | проверено по HANDOFF (PC/SP/PSW совпадают) |
| CMeP: keyring, Bigmac, Bignum, mailbox, strap, SC | `src/hw/cmep/` | Bigmac работает, Bignum в работе |
| Ernie: SFR, SC-протокол, eMMC-хост, питание | `src/hw/syscon/` | в работе |
| Kermit: GIC, таймеры, UART, SDIF, DMA, дисплей | `src/hw/soc/` | в работе |
| eMMC: карта + сборка образа из 1.04 | `src/hw/emmc/` | образ собирается, SLB2 читается с образа |
| Загрузчики: PUP/SLB2/SELF/ELF/ключи | `src/loader/` | 42/42 модулей, SELF→ELF проверено |
| Машина и цепочка загрузки | `src/machine/` | готово |
| Отладчик (CLI + API) | `src/debug/` | готово |
| SDL3-фронтенд | `src/ui/` | готово, оффскрин-скриншоты |
| Арт: иконка, логотип, талисман | `artwork/` | генерируется скриптом |

## Сборка

Требуется Visual Studio (MSVC) — скрипт сам находит тулчейн. SDL3 лежит в
`third_party/SDL3-devel-VC` (x64, версия 3.4.16).

```powershell
cd zeliboba
powershell -ExecutionPolicy Bypass -File .\build.ps1              # собрать всё
powershell -ExecutionPolicy Bypass -File .\build.ps1 -Clean       # с нуля
powershell -ExecutionPolicy Bypass -File .\build.ps1 -Tests       # + самотесты
```

Результат: `build/bin/zeliboba.exe` (консольный отладчик), `build/bin/zeliboba_ui.exe`
(SDL3), `build/bin/zlb_tests.exe`, `build/bin/emmc_rebuild.exe`, `build/bin/zdis.exe`.

### Visual Studio 2026

В корне лежит готовое решение — `zeliboba.slnx` (и эквивалентный классический
`zeliboba.sln`), CMake для работы в IDE не нужен:

```powershell
msbuild zeliboba.slnx -p:Configuration=Release -p:Platform=x64 -m
```

Стартовый проект — `zeliboba_ui`, так что **F5** сразу собирает и запускает
эмулятор. Список исходников в проектах задан масками, поэтому новые `.cpp`
подхватываются автоматически, а вывод идёт в тот же `build/bin`, что и у
CMake-сборки. Подробности — `msvc/README.md`.

## Запуск

```powershell
# интерактивный отладчик, машина уже сброшена и снабжена деталями
build\bin\zeliboba.exe

# выполнить команды и выйти
build\bin\zeliboba.exe -ex "boot" -ex "run 200000" -ex "regs" -ex "quit"

# начать сразу со второй стадии (second_loader) или с ядра
build\bin\zeliboba.exe --stage second -ex "run 100000" -ex "quit"
build\bin\zeliboba.exe --stage kbl    -ex "run 100000" -ex "quit"

# показать карту памяти и устройств
build\bin\zeliboba.exe --info

# SDL3
build\bin\zeliboba_ui.exe
```

## Отладчик

Полный список — `help`. Кратко:

```
step [n] | run [n] | runm [n] | until <addr>
core [mep|arm|rl78]        выбор активного ядра
regs | reg <имя> <знач>    регистры
dis [addr] [count]         дизассемблер
mem [addr] [rows]          дамп памяти
trace [n] | trace find <a> трейс шины
devices | devget | devset  MMIO-устройства и их регистры по именам
emmc info | emmc read ...  карта eMMC
keyring                    состояние keyring-контроллера CMeP
boot | stage <...>         цепочка загрузки
bp / watch / bpl / wpl     точки останова и наблюдения
log <level>
```

## Реконструкция eMMC

Образа eMMC нет. Образ собирается из распакованной прошивки 1.04:

* загрузочные разделы eMMC (`boot0`/`boot1`) и две копии в пользовательской области
  (`0x800000`, `0xC00000`) содержат контейнер **SLB2** с `second_loader.enp`,
  `secure_kernel.enp`, `kernel_boot_loader.self`, `kprx_auth_sm.self`, `prog_rvk.srvk`;
* пользовательская область — MBR + разделы `os0` и `vs0`, которые в PUP лежат
  готовыми образами FAT16 (`Out/PUP_dec/os0.bin`, `vs0.bin`).

Проверка: машина читает SLB2 **из собранного образа**, и все 7 записей совпадают
по имени, смещению и размеру с исходными файлами.

## Материалы

Эмулятор построен только на материалах рабочей папки и открытой документации на
устройство: анализ `dumps/bootrom_analysis/`, референсная реализация
`VitaTestSuite` (C#), `pup_fiction`, даташиты в `datasheets/`, `_rl78docs/`,
`ernie-master/`, `Sony_Vita_Sdk_0945-YLoD/`. Никакие сторонние эмуляторы не
использовались.

## Графика

Талисман проекта — **Зелибоба**, синий пушистый дух двора из российской
«Улицы Сезам»: длинная морда, висячие уши, тяжёлые брови, длинный конический
нос, белые кроссовки и коллекция галстуков. Портрет, иконка, логотип и баннер
целиком рисуются скриптом:

```bash
python3 artwork/make_artwork.py     # fonttools + cairosvg + Pillow
```

Иконка попадает и в `.exe` (через `src/zeliboba.rc` и `src/ui/zeliboba_ui.rc`),
и в окно SDL3 (`src/ui/zeliboba_icon_pixels.h` → `SDL_SetWindowIcon`). Палитра
взята из `src/ui/ui.h`, поэтому арт и интерфейс выглядят как одно целое.
Описание персонажа, состав файлов и палитра — в `artwork/README.md`.

## Документация

* `docs/STATUS.md` — состояние на сейчас: что работает, чего не хватает, подстановки
* `docs/ARCHITECTURE.md` — устройство эмулятора и контракты между модулями
* `docs/BOOT.md` — восстановленная цепочка загрузки 1.04 по шагам
* `docs/FIRST_LOADER.md` — CMeP first_loader по шагам: листинги, что делает код, что измерено
* `docs/KBL.md` — справочник по ARM-загрузчику `kernel_boot_loader`
* `docs/NSKBL.md` — справочник по небезопасному загрузчику ядра (NSKBL)
* `docs/SYSCON.md` — Ernie (RL78): SFR, SC/SPI, DRAM, JIG
* `docs/HARDWARE.md` — карты памяти и регистров, что подтверждено, а что гипотеза
* `docs/EMMC.md` — раскладка реконструированного образа
* `docs/DEBUGGER.md` — отладчик
* `docs/CPU_ARM_AUDIT.md` — аудит декодера ARM
* `docs/VENEZIA.md` — план по движку Venezia (MPE, IVC2)
* `docs/GPU.md` — план по PowerVR SGX543 и выводу Live Area
* `docs/DEVELOPMENT.md` — контракты для разработки
* `artwork/README.md` — талисман, палитра, состав арта и его генерация
* `msvc/README.md` — решение Visual Studio 2026

## Скриншоты

`zeliboba_ui` умеет снимать кадр без интерактивного рабочего стола:

```powershell
build\bin\zeliboba_ui.exe --screenshot out.bmp --screenshot-tab disasm --run 200000
python tools\bmp2png.py out.bmp out.png
```

Готовые кадры лежат в `screenshots/`.
