# Состояние проекта Zeliboba (эмулятор PS Vita)

Документ — сводка «где мы сейчас»: что в эмуляторе работает и чем это
подтверждено, чего не хватает до цели, какие места модели являются
**подстановками** (development substitution) и что делать дальше. Рабочий журнал
по звеньям цепочки — `docs/KBL.md` (раунды 1-43), по железу — `docs/HARDWARE.md`,
по eMMC — `docs/EMMC.md`, по отладчику — `docs/DEBUGGER.md`.

## 1. Цель

* **Stage 1** — загрузить ядро (os0) прошивки 1.04 на ARM (Kermit): цепочка
  Syscon → CMeP first loader → second loader → secure kernel → ARM
  (secure world) → NSKBL → `os0:psp2bootconfig.skprx` → SceSysStateMgr.
* **Stage 2** (после ядра) — движок Venezia MPE (реверс `vnzimg`) и PowerVR
  SGX543MP4+ на OpenGL с шейдерами, чтобы LiveArea отрисовала картинку.

Stage 1 **не достигнут**: ARM доходит до исполнения `kernel_boot_loader`, но
KBL не получает карту памяти (см. §3).

## 2. Что работает (и чем подтверждено)

| Что | Подтверждение |
|---|---|
| Сборка и самотесты | `build.ps1`; `zlb_tests.exe`: **210 прогонов / 15 падений / 22 ассерта** (базовая линия) |
| Реконструкция eMMC из PUP | `verify.ps1` → «verify OK»: 992 файла, 0 расхождений, MBR/SLB2/os0/vs0 |
| Первый загрузчик CMeP (MeP) | реальный дамп `dumps/vita_prototype_bootrom.bin` в окне 0x5C000; SUCCESS + передача на 0x40000 |
| Вторая стадия CMeP | исполняет SLB2-образ из eMMC (4121 чтение eMMC, idstorage в DRAM, SC-команды); заканчивается вызовом сервиса `jmp 0x5FF00` |
| Secure kernel (0x800000) | грузится по своему адресу компоновки, проходит стартовые проверки keyring/sysctl/версии, рукопожатие 0x9/0x101/0x102/0x106 |
| Отпускание ARM | по «done»-прыжку secure kernel: `release_soc()` → `ARM started on kernel_boot_loader at 0x40020000` |
| ARM KBL | исполняется на всех четырёх ядрах (барьер 0x4003B384), печатает баннер и «Safe Mode : [ YES ]» |
| Консоль прошивки | команда `console`/`uart` показывает вывод KBL (регистр данных `+0x70` блоков 0xE2030000/0xE2040000) |
| Отладчик | `run/runm/step/until/bp/bpc/bpl/watch/wpl/regs/dis/mem/poke/save/trace/devices/map/devget/devset/emmc/gpo/console/bootctx/boot/stage/keyring/info/load/log` |
| SDL3-фронтенд | `zeliboba_ui` (видео/звук/ввод), скриншоты через `--screenshot` |

Цепочка шагов 1-4 вики проходится **автоматически** в обычном прогоне (`boot` +
`runm`), без команд `stage`.

## 3. Чего не хватает до Stage 1

1. **ARM boot-контекст пуст.** Вторая стадия либо копирует контекст из DRAM
   (0x40000000) в scratch (`memcpy` 0x40A86), либо идёт по «холодной» ветке
   0x40858 — выбор делает бит 7 слова, полученного от syscon командой 0x0010
   (get_version), и в нашем прогоне он равен нулю; источник в DRAM при этом
   остаётся нулевым. В итоге KBL не получает карту памяти и умирает на первом
   исключении: `VBAR = 0x16100`, страница векторов не отображена (`ABT`,
   `pc=0x0001610C`).
2. **Ответ syscon «hardware info»** (0x446F2 → 0x44274 → 0x44246, команда
   0x0010 и соседние) содержит только то, что подтверждено вики; для сборки
   контекста нужны поля, которых у модели нет.
3. **Per-console ключи** (keyring 0x213) отсутствуют в дампах — из-за этого
   SMI/record/SCE-проверки второй стадии заменены подстановками (см. §4).

## 4. Подстановки (development substitution)

Все они отключаются переменной `ZLB_NO_SUBSTITUTION=1` (тогда поведение
воспроизводит нерешённую проверку и загрузка останавливается там, где
останавливалась бы без ключей).

| Место | Что сделано | Где |
|---|---|---|
| SMI: подпись, два keyed-контроля | ветки-проверки очищены | `src/hw/cmep/bigmac.cpp` |
| Configuration record: MAC-проверки 1-3, дайджест записи 0x0F | ветки очищены | там же |
| SCE: диспетчер команд и валидатор ответа | результат форсирован | там же |
| SMI RSA | результат форсирован | там же |
| ARM boot ROM | нет дампа → шаг реализован на C++ (стейджинг second_loader в SRAM + ящик 0xE0000010) | `src/machine/bootchain.cpp` |
| Сервис first loader'а `0x5FF00` | нет кода в дампах → перехват `pc_hook` и перезапуск CMeP в `secure_kernel.enp` | там же |
| Рукопожатие secure kernel | нет доставки прерываний в MeP → ответ ARM превращается в событие `state = 9` | там же |
| Отпускание ARM | нет SC-пути для сообщения → «done»-прыжок secure kernel вызывает `release_soc()` | там же |
| Зеркало scratch CMeP | ARM PA 0 = первые 32 КиБ scratch (вики) | там же |
| Развилка сборки ARM-контекста (0x40850) | ветка очищена, чтобы сборка выполнялась | `src/hw/cmep/bigmac.cpp` |
| Ключи CMeP | синтетическая таблица ключей + переподпись образа | `src/machine/bootkeys.cpp` |

## 5. Как запускать и проверять

```powershell
# сборка
powershell -ExecutionPolicy Bypass -File .\build.ps1
# самотесты (базовая линия 210/15/22)
.\build\bin\zlb_tests.exe
# полная проверка (сборка, раскладка, план загрузки, eMMC)
powershell -ExecutionPolicy Bypass -File .\verify.ps1
# прогон цепочки до ARM
.\build\bin\zeliboba.exe -q -ex "boot" -ex "runm 300000" -ex "core" -ex "console" -ex "bootctx"
# отключить все подстановки
$env:ZLB_NO_SUBSTITUTION=1
```

Полезные диагностические переменные: `ZLB_SDIF_TRACE`, `ZLB_BIGMAC_TRACE`,
`ZLB_SDIF_BYTE_ARG`, `ZLB_MEP_PC=addr[,addr…]`, `ZLB_WTRAP=lo-hi` (логировать
записи в диапазон), `ZLB_RTRAP=lo-hi` (чтения), `ZLB_RECORD_VERSION`, `ZLB_WTRAP`.

## 6. Дальнейшие шаги

1. **Достроить ARM boot-контекст**: разобрать «холодную» ветку 0x40858 (что она
   собирает и куда кладёт), при нехватке данных — заполнить источник контекста
   подстановкой; цель — чтобы KBL получил карту памяти и вектор 0x16100.
2. **Старт ARM с 0x00000000**, как в вики («reset the ARM CPU at 0x00000000»),
   вместе с зеркалом scratch на PA 0 (сейчас ARM стартует прямо с KBL).
3. **NSKBL → os0**: после KBL — `os0:psp2bootconfig.skprx`, `SceSysStateMgr`,
   загрузка модулей ядра из `SceKernelBootimage`.
4. **Stage 2**: Venezia MPE (`docs/VENEZIA.md`) и PowerVR SGX543MP4+
   (`docs/GPU.md`) на OpenGL с шейдерами, затем LiveArea.

## 7. Метрики на текущий момент

| Метрика | Значение |
|---|---|
| Самотесты | 210 прогонов / 15 падений / 22 ассерта (известные, унаследованные) |
| eMMC | 992 файла, 0 расхождений |
| Цепочка | шаги 1-4 вики автоматически; ARM запускает `kernel_boot_loader` |
| CMeP | вторая стадия ~600k инструкций, secure kernel ~78k инструкций |
| ARM | 4 ядра, барьер 0x4003B384 проходится; падение на векторе 0x16100 |
