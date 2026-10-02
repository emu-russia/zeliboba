# Состояние проекта Zeliboba (эмулятор PS Vita)

Сводка «где мы сейчас». В документе только то, что подтверждено прогоном,
дампом, дизассемблером или кодом модели; всё, что является допущением, собрано
в §4 и помечено как подстановка.

Разбор сессии от исходного A9 wait до гостевого логотипа, включая причины
5h37m работы и оставшиеся допущения:
[BOOT_LOGO_DEBRIEF_2026-10-01.md](BOOT_LOGO_DEBRIEF_2026-10-01.md).

## Текущая проверка (2026-10-02, WSL + VS2026)

Сборка — решение VS2026 (`msbuild zeliboba.slnx -p:Configuration=Release
-p:Platform=x64`), вывод в `build/bin`. Образ **`build/emmc.img` пересобран с
нуля** (`emmc_rebuild --firmware ../Vita_104_Firmware/Out --out build/emmc.img
--verify`): 992 файла, 0 расхождений, MD5 совпадает с прежним образом
(`b3e8f46f5e948bb7dcac06096b0bb014`), то есть содержимое детерминировано.
Самотесты: **629 прогонов, 0 отказов** (было 624; пять новых — ниже).

### Что исправлено в этой сессии

| Дефект | Проявление | Исправление |
|---|---|---|
| **Ядро дизассемблировало каждую инструкцию в горячем пути** | `ArmCore::step()` в конце **каждой** инструкции звал `arm_disassemble(*bus, cur_instr_addr_, …)`: чтение по **виртуальному** адресу через физическую шину (при включённом MMU это unmapped-обращение для любой инструкции ядра, а unmapped-байт шина достаёт перебором всех 83 устройств) плюс полное декодирование. Побочный эффект — `StepResult::text` был **мусором** везде, где VA ≠ PA. Цена: 200k слайсов = **34.6 s**, unmapped-обращений ARM ~1.2 на инструкцию | текст листинга стал opt-in (`Cpu::step_text`, по умолчанию **выключен**; включается там, где на него смотрят — трассировка и сообщение об undefined). Тот же прогон: **8.0 s (×4.3)**, unmapped-обращений почти нет. Поведение машины не изменилось: прогон `s600k → +400k слайсов` даёт состояние, отличающееся от прежнего `s1000k` **ровно на 22 байта счётчиков шины** (RAM, регистры, устройства — побитово те же) |
| `Debugger::step`/`history` игнорировали выбранное ядро ARM | `core arm3` + `step` крутил **arm0** (он в WFE) — команда «ничего не делала», а `history arm3` всегда отвечал «(history empty…)» | `step` резолвит `Arch::Arm` через `arm_core(arm_core_index_)`, как `active_core()` |
| `Vita::emulated_seconds()` читал `cycles` **arm0** | как только arm0 уходил в WFE, `boot`/`status_line` показывали одно и то же `t=0.279s` до конца прогона; `run_for()` не мог дойти до цели | часы машины — `KermitBlock::total_cycles()` (Kermit тикается ровно один раз на шаг/слайс) |
| `boot_.stage` не двигался дальше `arm-kernel-boot-loader` | стадии `NskblEntry`/`KernelEntry` ставились **только** отладочными `stage nskbl`/`stage kernel`, поэтому обычная загрузка после старта os0 всё ещё рапортовала загрузчик | `poll_boot_chain()` определяет NSKBL по PC в окне `0x51000000` и ядро по PC < `0x00800000` (с флагом `nskbl_seen_`, который входит в save state и сбрасывается на power-on reset) |
| Диагностика `ZLB_ARM_TRACE_RANGE` печатала мусор | каждая инструкция выглядела как `neon.dp 0xF3FFFFFF` / `.word 0xFFFFFFFF`: `ArmCore::disassemble` читал VA **напрямую через физическую шину** | `ArmCore::disassemble` транслирует байты через MMU и декодирует `arm_disassemble_bytes` |
| `sleep instruction` логировался как WARN | CMeP за загрузку засыпает ~33 раза, и каждый раз это предупреждение | архитектурный sleep — `DEBUG`; предупреждением остаётся только неожиданный halt (например, машинный хук) |
| `C4267` в `gic.cpp` (pulse-ядро) | предупреждение сборки | явное приведение |
| `Bus::read16` не сообщал о slow-path в траппу | 16-битные чтения MMIO/unmapped **не попадали в `ZLB_RTRAP`** (read8/32/64 и все записи попадали). Именно из-за этого гостевое чтение окна `0xE3320000` выглядело как «чтений нет», и диагноз несколько раундов шёл не туда | добавлен пропущенный вызов `note_read_trap` в slow-path `read16` (единственный асимметричный аксессор) |
| Окно `0xE3320000` (per-core control) отсутствовало | гость писал в немапированную периферию: 214 записей за холодную загрузку, `pc=0x51003328` (NSKBL) и `0x000EC39C` (ядро) | добавлен блок `Kermit.PerCore @ 0xE3320000` (0x4000) с регистрами `CORE0..3_00` и `CORE0..3_100`; семантика не восстановлена и это помечено как модель |
| Подстановка fault-окна применялась бы и в User Mode | user-адресное пространство **рандомизируется (ASLR)** и своё у каждого процесса; правило `PA = 0x40000000 + VA` наложило бы случайную физическую страницу на случайный VA и съело бы ожидаемый гостем abort | `satisfy_arm_boot_fault` выходит сразу при `mode() == kModeUser`; в обычном прогоне подстановки не срабатывают вовсе (`boot_fault_fixes_ == 0`), так что это подготовка к userland, а не изменение поведения |

Плюс `run-wsl.sh`: Windows-бинарник из WSL **не видит** переменные окружения
Linux-шелла без `WSLENV` — все `ZLB_*` диагностики молча не работали. Скрипт
сам добавляет их в `WSLENV` и запускает `build/bin/<ZLB_BIN>.exe`.

### Что гость делает с окном 0xE3320000

Измерено `ZLB_WTRAP`/`ZLB_RTRAP` по всей холодной загрузке: на каждое ядро
гость программирует `0xE3320000 + 0x1000*core + 0x100 = 1`, затем
`+ 0x000 = 0x87`, и **ни разу это окно не читает** (214 записей, 0 чтений).
Окно попадает в identity-секции TTBR1 (`0xE0000000..0xE7FFFFFF`), то есть
VA == PA. Что означает `0x87` — не восстановлено; блок только сохраняет записи,
чтобы они были видны в `devices`/`devget`, а не пропадали.

**Важно при работах на userland:** правка списка устройств шины меняет число
устройств в состоянии, поэтому старые сейвы перестают загружаться — это
проверяется (`state file: bus has 49 devices, this build has 50`) и печатается
как `loadstate failed`. Сейвы в `scratch/` после такой правки надо
перегенерировать (с ускорением ×4.3 это ~45 с).

### Экран

Сейв `s1000k` (`t = 0.769 s`) уже содержит кадр гостя: панель F7 показывает
белый логотип PlayStation на чёрном, 960×544 RGBA8888, stride 3840 —
[screenshots/save-state-ps-logo.png](../screenshots/save-state-ps-logo.png).
На `s600k` (`t = 0.46 s`) буфера ещё нет: IFTU сообщает «no supported enabled
guest-RAM scanout», и панель пишет «no framebuffer yet». То есть **логотип
появляется между `s600k` и `s1000k`**.

### Где на самом деле стоит загрузка

Измерено с сейвов (см. ниже), а не по журналам прошлых сессий:

* обычный холодный прогон проходит цепочку до **`stage=kernel-running`**;
  `boot` печатает `NSKBL entered at 0x51000000`, `kernel entered from NSKBL at
  0x000C43B2`;
* **образ eMMC читается только в разделе os0**: `lba=0..72800`, то есть
  `slot04/OS0` (`0x02000000`). Раздел **vs0** (`slot07`, LBA `0x0B000000/512 =
  360448`) не читается **ни разу**; счётчик `transfers` стоит на `10753 reads,
  0 writes`;
* до какого-то момента ядро ещё двигается: за 1M дополнительных слайсов из
  `s600k` (≈256M инструкций, `t` 0.77 s → 1.23 s) карта покрытия ARM выросла на
  **193 новых сайта** — страницы `0x004A2000..0x004AA000`, `0x004F4000..0x004F5000`,
  `0x005B7000` (планировщик и Display-producer);
* а затем система **полностью уходит в простой**. Замер с `s1000k` + 5M слайсов
  (≈12 минут машинного времени, `t` = **4.6126 s**): все четыре ядра в
  `halted=1 reason=wfe`, включая arm3, и **ни одного нового обращения к eMMC**
  (`10753 reads, 0 writes` — те же, что на `t≈0.3 s`), консоль пуста,
  `GIC pending=0`. То есть это не «медленно», а остановка;
* три ядра стоят в **idle-цикле ядра**
  `0x4A1248: push {r3,lr}; blx <veneer 0x4A43E4>; blx <veneer>; b 0x4A124A`,
  где veneer ведёт на `0x479698` = `wfe; bx lr` — то есть `for(;;) { WFE; WFE; }`
  (`CPSR=0x1F` SYS/ARM). Ядро не «крутится» и не ждёт прерывания от устройства:
  **у него нет готовых потоков**. arm3 до простоя крутился в delay-цикле
  `0x4F9DB8..0x4F9DCC` (`do { now = time(); } while (now - [r5+0x9C] <= 0xFA0)`);
* живыми остаются только периодические фоновые задачи: SPI0 к syscon
  (299 → 2213 передач за 3.8 s), DSI-кадры (23 → 254) и IFTU turnover между
  банками. Обменов с eMMC нет, `ZLB_SDIF_TRACE=1` не показывает ни одной новой
  команды, IFTU не опрашивается (5 чтений регистров за 20k слайсов);
* SDIF на границе оставляет неснятые `NORMAL_INT_STATUS=0xA` (TC+DMA) при
  `ERROR_INT_STATUS=0`, `CMD_INHIBIT=0` и `COMMAND=0`: стек хранения работу
  закончил;
* для справки о скорости: `t` = 0.769 s на `s1000k` (482M инструкций),
  ≈0.77 µs эмулированного времени на слайс, ≈6–7M инструкций/с интерпретатора.
  Дело, однако, не в скорости: за 3.8 s гостевого времени не сделано **ничего**.

Проверить «сколько файлов какого раздела гость реально прочитал» можно одной
командой: `python3 tools/emmc_read_map.py <log>` (журнал с `ZLB_EMMC_LOG=1`)
показывает 41 файл `KD/*.SKP` в os0 и **0** файлов в vs0.

Вывод для цели: LiveArea упирается не в отсутствие «поддержки userland» в
модели (её и не нужно — ядро само монтирует vs0 и запускает SceShell), а в то,
что **ядро заканчивает инициализацию модулей, уходит в idle и не доходит до
монтирования vs0**. Следующий измеримый признак прогресса — первое чтение
LBA ≥ 360448 (или строка в `emmc_read_map.py` по разделу vs0).

## Текущая проверка на macOS (2026-10-01)

Последняя полная проверка нативных CLI и SDL3: **617 тестов, 0 отказов**. Гостевой
белый PlayStation logo на чёрном фоне наблюдается в
native SDL capture (image omitted from source delivery). Исправлены флаги
Thumb внутри IT, carry у LSL и replicated-byte immediate, AP/AFE больших страниц ARM, отдельные MMU-банки
Secure/Nonsecure, ATS/PAR, вход в исключения и половинный адрес BLX. Монитор
исполняет настоящий код обработчика; старый хук больше не затирает построенные
прошивкой векторы исходной ELF-таблицей. Auth-файлы размещаются в DRAM после
IdStorage scratch, поэтому `kprx_auth_sm.self` сохраняет исходные байты.

Исправлены причины отказа настоящего TrustZone loader: хук больше не затирает
рабочий dlmalloc heap и заполненную гостем таблицу MemBlock; старые подстановки
page-cache и heap lookup выключены по умолчанию. Thumb32 `LDR.W PC` теперь
выполняет переход вместо ошибочной трактовки как preload. Гость сам выделяет
страницы, релокирует модули и запускает SceSysmem (`UID 0x20077`, start `0`),
SceExcpmgr (`0x20089`, `0`) и SceKernelIntrMgr (`0x2008F`, `0`). IntrMgr заменяет
содержимое MVBAR-векторов настоящими обработчиками: SMC `0x003BE1C8`.

Текущий обычный прогон `runm 1000000` проходит **0xA9**, читает следующие os0
modules: оба load lists возвращают `0`, все 28 modules получают положительные UID,
все native relocations успешны. Все 14 core-module starts, Stdio, Lowio,
Syscon, OLED, Display и SblSsSmComm возвращают `0`.
Прежние module-list failures
`0x803FF007` были вызваны A32 aligned/rotate word load, который портил `/kd/`
в `/os0` в native path normalizer. Common MemU исправлен для A32/Thumb16/Thumb32;
старый null sysroot callback/restart больше не является текущей границей.
Прежний NEON stop `0xF2201110` по `0x004A0008` исправлен: Q=0 допускает odd D
registers, Q=1 требует even indices; настоящий ThreadMgr initializer исполняется.
Прежний I2C wait устранён: два native controllers реализуют exact reset7/idle,
обоим гость делает настоящий reset. Остальные transfers остаются unsupported.
TPIDRURW/URO/PRW теперь сохраняются per-core и в отдельных Secure/NS banks;
native ThreadMgr context pointers больше не теряются, threads работают.
Lowio text находится по `0x005A8000`, его BSS выделен отдельно по `0x004F2000`.
Прежний NEON stop Syscon `0xFFC70E1F` исправлен: настоящий VMOV/VST1 padding
block исполняется. SceEmcTop `0xE8200000` завершает все шесть native commands;
unsupported operations остаются busy.
CDRAM `0x20000000..0x27FFFFFF` теперь имеет независимый RAM backing.
LT5 `0xE20B6000` теперь считает elapsed microseconds, настоящий GetSystemTimeLow
возвращает ненулевой счётчик. WT7 `0xE20BE000` доставляет IRQ135; настоящий
handler освобождает per-core wait, **SMC117 возвращает `0`**. Syscon дошёл до
SPI0 submission/GPIO3 rise и настоящий GPIO248/sub4 callback: reply прочитан,
ACK и stop выполнены гостем, 299 physical-ready edges в полном cold capture, RX пуст.
Исправлен RFE Thumb return на вторую halfword и User/System SP/LR alias;
прежние SP=0 и ложные memory faults `0x24/0x1CDA280` больше не наблюдаются.
Настоящий OLED A1 helper сохраняет SP и выдаёт точные 18 TX/RX bytes.
SPI2 input моделируется как undriven high, не как валидный panel ID: гость
отвергает `FFFF`, устанавливает ready2 и сохраняет API error `803F0A03`.
Display WaitReady, OLED script и head setup возвращают `0`; гость включает
DSI0, который даёт 23 frames в обычном cold capture. Исправлен настоящий Syscon
checksum block: VLD1/VMOVL/VADD/VPADD и scalar VMOV high-D исполняются.
Модель раннего handoff берёт boot bytes из того же Ernie NVS, что SC commands:
NVS4A0/481/483 → +30/+31/+33; +32=0. Cold profile — `FF FF 00 FF`, +6C=4;
неподтверждённый поздний external-boot marker удалён. Native allocated per-core
stacks больше не сдвигаются старым shared-stack workaround (fallback только4000).
Все четыре настоящих Display boot predicates возвращают0. Native producer
`0x5B7048`, gzip `0x5B70AC→70B0` возвращает `0x1FE000` в VA10000000/PA1C000000,
SetFrameBuf и vblank wait возвращают0. Все2088960 guest bytes совпадают с embedded
logo SHA256 `80c43bdc43d6fcc7f1419960726e6dc3e6be906ae0faac2cdf060204717a6979`.
IFTU deferred publication теперь проходит: следующий DSI frame меняет current
на bank1 и поднимает physical IRQ204. Настоящий Lowio handler `0x5AD99C`
выполняет zero ACK, +180 rearm и replay того же PA в старый bank0; guest pending
очищается самим гостем. Native SDL показывает white PlayStation logo, 960×544,
pitch3840. Offline asset не инжектирован. Ordinary `runm 1000000` даёт 23 DSI
frames и enabled bank1 scanout; image clean, 10753 reads/0 writes.
Свежий прогон: `build/goal-native-iftu-arm-full.log`, breakpoint evidence:
`build/goal-arm-native-syscon-oled-iftu-arm.log/.json`; сводка
[integrated evidence](../build/goal-native-iftu-arm-integrated-evidence.md).
Прежний cold-stack capture без framebuffer сохранён как отрицательная проверка
до реализации turnover. Один future frame на full +180=1 и внутренний IRQ latch
при raw +40=0 — явно выбранная ограниченная модель, не восстановленная физическая
status encoding. Full fade, blending, полная загрузка ядра и SGX остаются открыты.
Первый GPIO revision ошибочно исключал early second-loader reply; этот regression
исправлен настоящим SPI-ready edge и покрыт unchanged MeP QueryIntr/ACK test.
Все Lowio initializer returns и реальные mutex/event UIDs подтверждены
отдельными breakpoint captures. Module-start `0` не означает завершение workers.
CMeP cleanup возвращается в native sleep `0x80048A`.
Прежний Smsched panic после polling ACK устранён.
Настоящее рукопожатие **0x101 → shared PA 0x401402C0 | 1 → 0x102** завершено.
Удалены принудительный CMeP state `9` (запрос shutdown) и ошибочный признак
успеха по PC `0x40002`; модель отпускает ARM после настоящего статуса `0x101`,
оставляя обмен scheduler гостю. Mailbox endpoints теперь сохраняют PA при
doorbell и очищают подтверждённые входящие биты.

Добавлены save states (`savestate`/`loadstate`): снапшот содержит RAM и общие
буферы, регистры и внутреннее состояние всех MMIO-устройств, четыре ARM-ядра,
CMeP, Ernie/RL78, keyring и флаги цепочки загрузки. Продолжение после
`loadstate` совпадает с непрерывным прогоном бит-в-бит, включая загрузку
снапшота в новом процессе (проверено на 300k и 1000k слайсах: состояния после
одинакового продолжения побайтово равны). Тесты — **624, 0 отказов**; контракт —
[SAVE_STATE.md](SAVE_STATE.md), команды — [DEBUGGER.md](DEBUGGER.md).

Mailbox IRQ доставляется в CMeP source 8: реальный вектор `0x800050` ведёт
в обработчик, который подтверждает и выполняет **0x80A01** для RVK initialization.
`prog_rvk.srvk` побайтово совпадает с исходной прошивкой. Реализованы настоящие
DMA copy, AES-256-CBC `0x238A`, AES-128-CBC `0x218A` и SHA-256 `0x2093`.
Native RVK RSA-проверка теперь проходит: прежняя ошибка `0x800F0024` исчезла.
Plain-section copy `0x2080`, HMAC-SHA256 `0x20B3` и AES-128-CTR `0x21A1`
также исполняются с настоящими результатами. **RVK callback возвращает `0`**;
гость публикует 21 запись (`0x807698 = 0x15`, `0x80769C = 0x809410`).
CTR plaintext побайтово совпадает с независимой распаковкой, counter обновлён
на 42 блока. Оба section HMAC проходят; прежняя ошибка `0x800F0627` устранена.
Независимо от RVK настоящий `0x80901` регистрирует буфер `{0x40140200, 0x80}`;
SceSblSmsched (`UID 0x200AF`) возвращает start `0`, поле готовности `0x5190A0 = 1`,
все SMC `0x12D..0x13C` зарегистрированы гостем.

Исправлен повторный дефект проверки runtime-векторов: relocated IntrMgr handler
`VA 0x003BE1C8` проверяется через Secure translation (`PA 0x401941C8`), поэтому
его таблица больше не затирается ELF-страницей. В предыдущем прогоне до
исправления GIC NSKBL проходил настоящие SMC,
определяет MMC и читает os0 через ADMA (LBA 0, 65536, 65568, 72800, по 32 сектора).
Текущий прогон даёт **10753 чтения секторов, 0 записей**, образ не изменён.
CMeP получает настоящий запрос для `kprx_auth_sm.self`: parser и начальный
policy возвращают `0`, RVK доступен. Добавлен явно моделируемый public retail
identity prefix в slot `0x509`; индивидуальные console bytes остаются нулевыми.
Настоящий platform/revocation policy `0x8047B8` теперь возвращает `0` вместо
прежнего `0x800F0B31`. Payload CTR и streamed HMAC `0x24B3/0x2CB3/0x28B3`
исполняются с настоящими результатами: native first chunk 28544 bytes,
final chunk 52 bytes. Payload return **`0`**, outer launch **`1`**;
CMeP достигает подлинного entry **`0x80B000`** с context `0x401402CC`.
28596 plaintext bytes совпадают с ELF PT_LOAD, HMAC digest — с metadata.
MeP SWI delivery теперь исправлена по manual: native vector `0x800014`
регистрирует IRQ9 callback `0x80E55E`, syscall возвращает `1`, модуль остаётся
в work loop. Прежний premature terminal sleep `0x80B050` устранён.
DRAM aperture slot `0x513` (`0xE0062260..263`) теперь выдаёт уже заданные
board 512 MiB. Byte/word accesses согласованы, raw overrides сохраняются,
reset восстанавливает профиль; native bounds checks остаются настоящими.
Прежний transport `5` / ARM `0x800F052A` устранён.
**Первый os0 файл `psp2bootconfig.skprx` проходит настоящие сервисы
`0x10001/0x20001/0x30001`: transport `1`, service return `0`.** Metadata CBC/SHA,
section CTR и streamed HMAC проверены гостем. Его 733-byte zlib stream
распаковывается независимо в 1624 bytes, совпадающие с ELF PT_LOAD0.
Прежний отказ native ARM inflater `0x51024E18 → 0x51024E8C` исправлен:
Thumb32 word load ошибочно округлял unaligned source вниз. Теперь accesses
адресуются точно, с per-byte MMU translation, alignment/attribute faults и
endianness. Настоящий decoder без изменения firmware возвращает `0x658` и
выдаёт все 1624 исходных ELF bytes. Результат не подставляется.
Native stop принимается и публикует exit `0x10000`. Cleanup вызывает
uncached helper `0x400CE`, чьи подлинные bytes есть на `0x8000CE`. Board handoff
теперь переключает low CMeP SRAM на тот же backing, helper исполняется;
warm/cold reset восстанавливает loader backing и bus fast paths.
Реальный selecting register remap пока неизвестен: это явно обозначенная модель.
В полном прогоне оба native inflater returns успешны: `0x658` (ELF PT_LOAD0)
и `0x318` (relocation segment), побайтово совпадают с подлинным ELF.
Все три segment loads, native relocation и первый callback завершаются с `0`.
Прежний CMeP undefined `0x800002` оказался ошибкой Bigmac opcode dispatch:
native zero-fill `0x000C` был принят за legacy AES и затирал kernel через low SRAM
alias. Теперь captured source0/destination+04/length+08/fill+34 форма исполняет
точный zero-fill с проверкой всего диапазона; stale IV/key не используются.
Настоящие uncached helper, restart и sleep `0x80048A` проходят без ошибки.
Прежний отказ двух module-list loads `0x803FF007` исправлен архитектурным
word access: A32 `E4913004` больше не копирует `/os0` вместо `/kd/` из unaligned
source. Native FAT/path execution снова читает os0; callback и module returns
не подставляются. Все 14 core-module starts, Stdio и Lowio успешны; следующий
Secure hardware polling описан выше. All 28 referenced SELF/ELF доступны и загружены.

Native trace выявила прежние дефекты GIC: неверная база и перекрытие L2CC,
128 источников вместо требуемых 256, неверный шаг priority/target/config.
Они исправлены вместе с завершением PL310 maintenance-команд. Прошивка
задаёт Secure Group0 для mailbox `200..203`, CPU CTLR `9` (FIQEn), target `8`
(ARM3). Реализованы per-core/security views и FIQ delivery; Nonsecure init
сохраняет Secure enables. Исправлена защита `SCR.FW/AW` для CPS/MSR/exception
return: Nonsecure `CPSID IF` больше не меняет запрещённый F-bit.
Текущий FIQ подтверждается настоящим ARM3 IAR. Исправлен SPI reset по Cortex-A9:
ICDICFR `0x55555555`, level-sensitive trigger, fixed handling-model bit.
После polling ACK pending снимается; прежний handler со status `0` и panic
(`smsched.c:1948`) больше не вызывается. Новый status вызывает настоящий FIQ.
Отдельно исправлен Thumb16 `MOV PC,Rm`: сохраняет Thumb и вычисленный target;
прежний undefined на `0x000C1C22` исчез.

Прежний прогон до A9 проходил дальше после ошибок загрузки модулей TrustZone,
поэтому не подтверждает рабочий scheduler. В нём исправленный SDIF уже позволял
гостю пройти CMD8/CMD5/CMD55, определить MMC и прочитать os0 через настоящую ADMA
(LBA 0, 65536, 65568 и 72800, по 32 сектора; 8345 чтений, 0 записей).
CommandTimeout, error signal enable, Error Summary и отмена timeout при reset
остаются исправлены. Прежние ошибки UID/page-cache устранены текущими изменениями.

Запуск настоящих os0 kernel modules начат, полная загрузка ещё не завершена;
гостевой логотип PlayStation наблюдается в native SDL. Исторические цветные полосы
SDL проверяли синтетический scanout. Исправлена передача формата пикселей
отдельно от padded pitch. Реверс настоящих `display.elf/lowio.elf` дал
IFTU0 `0xE5020000` и DSI0 `0xE5050000`, IRQ213/sub1; минимальный IFTU0 уже
читает framebuffer из guest RAM. Progressive VIC0 DSI0 timing и IRQ213 delivery реализованы и проверены;
bank turnover/IRQ204 реализованы в ограниченном ordinary mode01 subset;
blending ещё не завершён. Настоящий embedded logo теперь
распакован guest producer и опубликован IFTU
в scanout. Контракт и границы модели — [FIRMWARE_DISPLAY_104.md](FIRMWARE_DISPLAY_104.md).
Подстановки отсутствующего ROM/boot-контекста остаются частью модели.
Текущие журналы: `build/goal-native-timers-tests.log`,
`build/goal-native-timers-full.log`, `build/goal-native-timers-integrated-evidence.md`,
`build/goal-native-timers-graphics-evidence.md`,
`build/goal-native-emc-syscon-tests.log`,
`build/goal-native-emc-syscon-full.log`, `build/goal-native-syscon-neon-cpu-fix.md`,
`build/goal-cdram-board-mapping.md`, `build/goal-native-graphics-560-evidence.md`,
`build/goal-native-thread-context-tests.log`,
`build/goal-native-thread-context-extended.log`, `build/goal-native-thread-context-full.log`,
`build/goal-native-e820-emctop-evidence.md`, `build/goal-native-syscon-registry-review.md`,
`build/goal-native-tpidr-cpu-fix.md`,
`build/goal-debug-arm-inspection-fix.md`, `build/goal-native-lowio-i2c-evidence.md`,
`build/goal-native-neon-full.log`, `build/goal-native-neon-cpu-fix.md`,
`build/goal-native-memu-full.log`, `build/goal-native-path-cpu-fix.md`,
`build/goal-native-deflate-cpu-fix.md`,
`build/goal-integrated-bootconfig-load-evidence.md`,
`build/goal-arm-native-bootconfig-list-results.log`,
`build/goal-native-rvk-ctr-result-capture.log`,
`build/goal-native-kprx-stream-hmac-result-launch-capture.log`. Прежний panic:
`build/goal-native-smsched-panic-cmp.log`. Предыдущие: `build/goal-native-cpsid-entry.log`,
`build/goal-native-presmc103-vectors.log`.
Прежний путь A9: `build/goal-timeout-full.log`.

Старые раунды ниже сохранены для истории; их диагнозы и opt-in исправления
не заменяют текущую трассу и не должны считаться доказательством загрузки ядра.

Карта документации:

* `docs/BOOT.md` — цепочка загрузки 1.04 по шагам: что подтверждено, что модель;
* `docs/FIRST_LOADER.md` — CMeP first_loader по шагам с листингами и измерениями;
* `docs/KBL.md` — справочник по ARM-загрузчику `kernel_boot_loader`;
* `docs/NSKBL.md` — справочник по небезопасному загрузчику ядра (NSKBL);
* `docs/SYSCON.md` — Ernie (RL78): SFR, SC/SPI, DRAM, JIG;
* `docs/HARDWARE.md` — карты памяти и регистров;
* `docs/EMMC.md` — раскладка реконструированного образа;
* `docs/DEBUGGER.md` — отладчик и диагностические переменные;
* `docs/CPU_ARM_AUDIT.md` — аудит декодера ARM;
* `docs/SMC_AUTH_104.md` — подтверждённый ABI secure-module scheduler/auth
  прошивки 1.04; справочник для следующего шага, не свидетельство загрузки ядра;
* `docs/VENEZIA.md`, `docs/GPU.md` — план Stage 2.
* `docs/GRAPHICS_REFERENCES.md` — индекс локальных Graphics PDF (15 файлов,
  800 страниц), точные ссылки и пробелы между SDK API и текущей моделью GPU.
* `docs/FIRMWARE_DISPLAY_104.md` — настоящий 1.04 logo producer, IFTU/DSI
  registers и IRQ path; offline asset не подтверждает guest presentation.

Журналы старых раундов, из которых собраны `KBL.md` и `NSKBL.md`, не включены
в исходный архив; история git в этой локальной копии отсутствует. Ссылки вида
«`docs/KBL.md` round N» разрешаются по «Индексу раундов» в справочнике.
Новые macOS/native captures и отчёты сохранены в `build/` по указанным выше путям.

## 1. Цель

* **Stage 1** — загрузить ядро (os0) прошивки 1.04 на ARM (Kermit): цепочка
  Syscon → CMeP first loader → second loader → secure kernel → ARM
  (secure world) → NSKBL → `os0:psp2bootconfig.skprx` → SceSysStateMgr →
  модули `os0:kd/`.
* **Stage 2** (после ядра) — движок Venezia MPE (реверс `vnzimg`) и PowerVR
  SGX543MP4+ на OpenGL с шейдерами, чтобы LiveArea отрисовала картинку.

Stage 1 **не достигнут**. Текущий обычный прогон (`boot` + `runm`, без команд
`stage`) запускает настоящие модули TrustZone и scheduler, достигает NSKBL/A9
запускает `kprx_auth_sm`, проходит first os0 authentication/decryption,
native decompression/relocation/linking и bootconfig start `0`.
После исправления A32 path copy, NEON и TPIDR загружает все 28 list modules,
запускает все 14 core modules, Stdio, Lowio, Syscon, OLED, Display и SblSsSmComm.
Native threads работают; LT5/WT7 позволяют завершить все шесть EMC commands
и настоящий SMC117. RFE context и GPIO/SPI ready исправлены; Display включает
DSI0. Syscon checksum SIMD исправлен; гость распаковывает logo и передаёт
framebuffer. IFTU deferred bank publication и SDL logo подтверждены выше;
дальнейшая работа — продолжение kernel boot и восстановление полного hardware protocol.
Прежний путь обходил ошибки TrustZone. Полный Stage 1 не завершён.

## 2. Что работает (и чем подтверждено)

| Что | Подтверждение |
|---|---|
| Сборка | Windows `build.ps1`, macOS `build-macos.sh` → CLI, SDL3, `zlb_tests`, `emmc_rebuild`, `zdis` в `build/bin` |
| Самотесты | нативный `zlb_tests`: **617 прогонов, 0 падений** |
| Реконструкция eMMC | `verify.ps1` → `emmc_rebuild --verify`: **992 файла, 0 расхождений** (os0 63/63, vs0 929/929 файлов) |
| CMeP first loader | реальный дамп `dumps/vita_prototype_bootrom.bin` в окне `0x5C000`; сообщает `first loader reported SUCCESS to the ARM mailbox` и передаёт управление образу на `0x40000`. Вторая сборка — `dumps/pch-5c-cold_first_loader.bin` (retail PCH): `--first-loader` исполняет и её, раскладка констант определяется автоматически (`detect_first_loader_layout`, сдвиг `.data` −0x80) |
| CMeP second loader | исполняется из `0x40000` после настоящего AES-128-CBC (ключ — слот keyring 10, `ENC_KEY‖ENC_IV`); ~4121 чтение eMMC; заканчивается вызовом сервиса `jmp 0x5FF00` |
| CMeP secure kernel | загружается на `0x800000`, проходит настоящее `0x101/0x102`, принимает mailbox IRQ; RVK/auth, CBC/CTR/SHA/HMAC проверены, `kprx_auth_sm` запускается на `0x80B000`; SWI/IRQ9, first os0 auth/decryption успешны; native zero-fill, uncached SRAM helper и cleanup/restart/sleep исправлены |
| ARM KBL | работает на всех четырёх ядрах, печатает настоящий баннер; текущий путь загружает и запускает модули TrustZone, включая SceSblSmsched со start `0` |
| NSKBL | гостевой ARZL/filter, VA `0x51000000`, `0xA9`, настоящий SDIF/os0/kprx; все 28 list modules загружены, все 14 core starts и шесть driver starts `0`; настоящий logo producer/gzip/SetFrameBuf, IFTU IRQ204/ACK/replay и белый logo в SDL подтверждены |
| Syscon (RL78) | прошивка `ernie-master/USS-1001.bin` исполняется ядром RL78; SC-команды обслуживаются (get_status, get_version, get_rtc, get_gauge_status, nvs_read, scratchpad_read, set_power_hold, …) |
| eMMC-хост | SDHCI `0xE0B00000`: CMD0…CMD8, CMD16/CMD17/CMD18, ADMA2, R1b, command timeout; четыре ADMA-передачи по 32 сектора при чтении os0 |
| Консоль прошивки | команды `console`/`uart`; регистр данных `+0x70` окон `0xE2030000`/`0xE2040000` |
| Отладчик | `run/runm/step/until/bp/bpc/bpl/watch/wpl/regs/dis/mem/poke/save/trace/devices/map/devget/devset/emmc/gpo/console/bootctx/faults/boot/stage/keyring/cov/info/load/log` |
| SDL3-фронтенд | `zeliboba_ui` (видео/звук/ввод), оффскрин-скриншоты через `--screenshot` |

## 3. Чего не хватает до Stage 1

Нумерация пунктов сохранена от прежней редакции, чтобы ссылки в комментариях кода
(например, «`docs/STATUS.md` 3.1») оставались осмысленными.

0. **Исторический диагноз первой внешней загрузки NSKBL**. Текущий stop scheduler
   описан в начале файла; дальнейший текст относится к прежним раундам.
   NSKBL доходит до чекпойнта `0xA9` («kernel pre-init done, before first external
   load»): том и каталог разобраны, `psp2bootconfig.skprx` найден, но **данные
   файла не читаются ни разу** (за прогон 22 чтения eMMC, первый кластер файла не
   запрашивается). Ожидание `0x5101FE60` изымает узлы из списка завершений
   устройства по VA `0x2640` (`[объект]+0x2400`, объект — `0x240`), а список всегда
   пуст: структуру устройства `VA 0x240` никто не заполняет (ноль записей в её
   первой странице за весь прогон). Пустой список ведёт на `blx 0` в `0x5101D6A4`
   и дальше в `0x510205D6`, где собирается `0x80320011`; загрузчик возвращает его
   и arm0 крутится в `0x51000D0C`.

   **Уточнено раундами 380–391** (`docs/NSKBL.md` §7): стена названа по имени —
   задача внешней загрузки `0x510014D4` открывает устройство `os0:` (`0x51001104` →
   сервис `0x10005` в диспетчере `0x51018F6C`), и это открытие падает. Подстановка
   `ZLB_NSKBL_DEV=1` (по умолчанию выключена) теперь ставит
   `[device+0x2410] = 1` — **единственное поле, которое цепочка `0x5101EE98`
   проверяет перед тем, как отдать запрос устройству** (`0x5101EF20`: при нуле
   возвращается `0x8032001A`). Измеренный эффект этой правки: чтение файла впервые
   **доходит до контроллера** (`CMD18 arg=0x0238C000`, LBA 72 800), чтений карты
   становится 23, покрытие ARM растёт с 20 552 до **21 106** сайтов (+286), в
   консоли гостя появляются строки прогресса.
   Дополнительно `ZLB_NSKBL_DMA_FIX=1` (вместе с `ZLB_NSKBL_DEV=1`) достраивает
   параметры передачи в точке вызова метода хранилища (`0x5101DAE0`, узел в `r0`):
   размер `768 → 512`, аргумент-смещение `0x0238C000 → LBA 72 800`, пустая таблица
   ADMA `→ 0x510FF000`. Измеренный эффект: **данные первого кластера файла впервые
   читаются успешно** — `mmc read lba=72800 count=32 ok=1`, а диагностика
   `0x8032001a 1169 0 0x11c60(72800) 32` из консоли гостя **исчезает** (ноль строк
   `0x8032xxxx`).
   Остаётся открытие `os0:`: ветка успеха `0x5100151C` и монтирование `0x510012F4`
   не исполняются, ветка отказа `0x5100153E` — исполняется, поэтому чекпойнт всё ещё
   `0xA9`. Опрос в асинхронном пути (`0x510207B2`, 64 попытки) считает завершением
   отрицательное слово в `[node+0x10]`, а его узлы приходят из собственного
   аллокатора драйвера — дописать их поля вслепую нельзя (проверено: такие правки
   либо no-op, либо ломают рабочий путь чтения тома).
1. **ARM boot-контекст модель не воспроизводит.** Вторая стадия либо копирует
   контекст из DRAM (`0x40000000`) в scratch (`memcpy` `0x40A86`), либо идёт по
   «холодной» ветке `0x40858`; выбор делает бит 7 слова, полученного от syscon
   командой `0x0010` (`get_version`), и в измеренных прогонах он равен нулю, а
   источник в DRAM остаётся нулевым. То, что kernel_boot_loader читает по PA 0,
   модель получает зеркалом scratch CMeP (подстановка, §4).
   **Измерено покрытием и трапом:** запись `SceKblParam` (PA `0x1F000100`) в
   обычном прогоне **нулевая** — вторая стадия исполняет только её обнуление
   (`0x41B38`), до сборщика (`0x41B4A`) не доходит, а C++-сборщик при включённых
   подстановках не вызывается. KBL читает нули (`0x400376E4`, `0x4002028C`) и
   всё равно доходит до Non-Secure, то есть запись не является условием
   перехода в NSKBL.
   **Проверка гипотезы («дело в пустом контексте») — снята:** с
   `ZLB_CMEP_HANDOFF_RUN=1` вторая стадия доигрывает ветку (473 206 774 инструкции,
   впервые исполнены `0x40858`/`0x403C0`) и запись по PA `0x100` появляется —
   но NSKBL стоит на том же `0xA9`, а структура устройства по PA `0x40300240`
   остаётся нулевой (`docs/NSKBL.md` §7, раунд 374).
2. **Раздел памяти KBL приходит наполненным от предыдущей ступени.** В самом KBL
   все 15 обращений к аллокатору идут с флагом `0x10` («только кэш»), поэтому
   нарезка новых блоков запрещена, кэш блоков `0x1000` за 230 млн инструкций так
   и остаётся пустым (37 записей — все из инициализации самого раздела), и
   выделение возвращает `0x80020005`. Дальше по цепочке это держит создание
   менеджера объектов и регистрацию классов; в модели кэш выдаётся подстановкой
   (§4).
3. **Менеджер объектов** (`boot context +0x8C`) в модели создаётся только
   благодаря подстановке кэша блоков; собственного пути наполнения раздела у KBL
   нет (см. п. 2).
4. **Ответ syscon «hardware info»** (команда `0x0010` и соседние) содержит только
   подтверждённые вики поля; полей, из которых собирается ARM boot-контекст, у
   модели нет.
5. **Окно `0xECxxxxxx`** (Pervasive/DMAC и прочее), в которое KBL пишет и читает
   (`pc=0x400209E4…0x40020A36`), в модели ARM отсутствует — чтения дают 0.
6. **Per-console ключи** (keyring `0x213`) в дампах отсутствуют — из-за этого
   SMI/record/SCE-проверки второй стадии CMeP заменены подстановками (§4).

## 4. Подстановки (development substitution)

Основной выключатель development-патчей и перехватов — `ZLB_NO_SUBSTITUTION=1`;
он сохраняет нерешённые проверки, на которых останавливается строгий прогон.
Это не отключение всех моделей отсутствующего железа: роль ARM boot ROM,
board-профиль и аппаратные устройства остаются моделью; provisioning ключей
отдельно управляется `--no-provision`. Ниже удалённые и opt-in эксперименты
помечены отдельно от текущих fallback-подстановок.

| Место | Что сделано | Где |
|---|---|---|
| SMI: подпись, два keyed-контроля | ветки-проверки очищены | `src/hw/cmep/bigmac.cpp` |
| Configuration record: MAC-проверки 1-3, дайджест записи `0x0F` | ветки очищены | там же |
| SCE: диспетчер команд и валидатор ответа | результат форсирован | там же |
| SMI RSA | результат форсирован | там же |
| ARM boot ROM | дампа нет → шаг реализован на C++ (стейджинг `second_loader.enc` в DRAM, ящик `0xE0000010`) | `src/machine/bootchain.cpp` |
| Сервис первой стадии по `0x5FF00` | кода нет **ни в одном** дампе (прототип — нули, retail-снимок — таблица указателей на затёртый `.bss`); вторая стадия сама пишет туда 32-словный дескриптор и уходит `jmp` → перехват `pc_hook` и перезапуск CMeP в `secure_kernel.enp`. Передача «первый → второй» (шаг 15) при этом **честная**, хук стоит только на обратном вызове | там же |
| Рукопожатие secure kernel (старое, **удалено**) | fake `state = 9` и автоматический ответ ARM `1` больше не используются. Native handshake: `0x101` → гостевой PA `0x401402C0` и doorbell `1` → `0x102`; endpoint ACK и дальнейшие ответы обслуживает гость | `src/machine/vita.cpp`, `src/hw/cmep/mailbox.cpp` |
| Пробуждение CMeP (старое, **удалено**) | принудительное снятие `halted` после fake события удалено; sleep/IRQ wake и IRQ/RETI/SWI теперь выполняются через архитектурный CPU/INTC путь | `src/cpu/mep/mep_core.cpp` |
| Отпускание ARM | модель вызывает `release_soc()` после настоящего статуса secure kernel `0x101`, сохраняя его для guest ACK. Старый «done» по прыжку в `0x40000..0x5BFFF`/PC `0x40002` удалён | `src/machine/vita.cpp` |
| Низкое окно ARM (VA < 1 МиБ) | при translation fault модель дописывает отображение в L2 ядра и повторяет доступ; правило по умолчанию `window` (`VA 0x40000 → PA 0x40118000`), `ZLB_ARM_LOW_MAP=identity\|dram\|dram-abs` дают альтернативы. Нужна только secure-этапу: собственные таблицы KBL не описывают `L2[0x00]`, `[0x01]`, `[0x10]`, `[0x19]`, `[0x1A]` (замеры — `docs/KBL.md` 7.1) | `src/machine/bootchain.cpp` + `ArmCore::fault_hook` |
| Распарковка барьера и WFE-кластера (legacy opt-in) | Старые искусственные tick/counter/wake эксперименты — только `ZLB_KBL_WFE_TICK=1` / `ZLB_KBL_WFE_WATCHDOG=1`. Native SEV и interrupt delivery достигают настоящего bootconfig start `0` с обоими механизмами выключенными: `cluster_wakeups=0`, `counter_patches=0`, `irq_wakeups=7`; `build/goal-native-no-wfe-watchdog-full.log`. Ранние замеры `docs/KBL.md` 7.1.8–7.1.39 описывают прежнюю модель | `src/machine/vita.cpp` (`run_slice`) |
| Страница векторов `PA 0x16100` | начальные 0xC0 байт векторов KBL (сегмент `vaddr=0` его ELF) ставятся перед стартом; последующая корректная Monitor-страница, построенная гостем, сохраняется | `src/machine/bootchain.cpp` |
| Кэш блоков раздела KBL (legacy opt-in) | старый stand-in выдаёт блоки `0x1000` при `ZLB_PART_BLOCK_CACHE=<n>`; текущий default **0**, прежний default 4 относится к старым сериям | там же |
| Таблица классов по глобалу `0x400B291C` | ранний fallback создаёт девять пустых записей только при нулевом указателе (`ZLB_KBL_CLASS_TABLE=0` отключает). Native KBL сам выделяет и заполняет таблицу; текущий хук сохраняет её ненулевой указатель и содержимое, не создаёт native free-pool объекты | там же |
| Кука/арена экземпляров, низкое окно кучи, стек ядра | `ZLB_KBL_*` fallback-гейты (§5); валидные native dlmalloc heaps с собственной cookie/arena сохраняются. Старый heap-lookup shortcut — opt-in `ZLB_KBL_HEAP_LOOKUP=1`, default **0** | там же |
| Ключи CMeP | синтетическая таблица ключей `0xE0066000` + переподпись staged-образа (`--no-provision` отключает) | `src/machine/bootkeys.cpp`, `tools/make_boot_keys.py` |
| Структура устройства NSKBL (legacy opt-in) | `ZLB_NSKBL_DEV=1`, по умолчанию **выключена**. Текущий прогон создаёт настоящий объект драйвера и читает первый os0-файл без этого stand-in; ранние неудачные опыты — §3.0 | `src/machine/bootchain.cpp` |
| KBL параметры холодной загрузки | существующий modeled handoff теперь читает NVS4A0/481/483 для +30/+31/+33; +32=0, retail product +6C=4. Native per-console builder полностью не исполняется. Поздний marker +33FF/+6C bit0 и `ZLB_NSKBL_BOOTCFG` **удалены**: bit0 обозначает external boot | `src/machine/bootchain.cpp`, `docs/KBL.md` §6 |
| Shared KBL stage stack | исторический fallback сдвигает только исходный top4000. Native 16KiB per-core allocations сохранены; прежний безусловный bias выходил за allocation и терял saved PC | `src/machine/bootchain.cpp`, `build/goal-native-cold-stack-handoff-review.md` |
| Пул/куча/магия класса NSKBL | `ZLB_NSKBL_POOL`, `ZLB_NSKBL_PHYSPOOL`, `ZLB_NSKBL_LOWWIN`, `ZLB_NSKBL_MAGIC`, `ZLB_NSKBL_POOLFIX` | там же |

Ранее существовавшая подстановка «ARM boot-context gate» удалена: она патчила
`0x80850` (адрес записан как абсолютный, а цикл ещё раз прибавлял базу `0x40000`
— немапированная память, то есть не делала ничего) и моделировала *resume* вместо
холодной загрузки. В таблице подстановок теперь есть проверка «адрес патча не
замапирован → warn», а тест `second_loader_substitutions_stay_inside_the_staged_image`
не даёт внести такую запись снова.

**Удалено в этой серии (измерением, а не догадкой):**

* подстановка halfword-хелпера `0x4003A3EC` (замена `ldrexh/strexh` обычной записью) —
  трасса монитора эксклюзивов (`ZLB_EXCL_LOG=1`) показала 1797 событий без единого
  отказа, а барьер доходит до `0xA9` и без подстановки. Старое поведение осталось за
  `ZLB_KBL_LOCK=1` только для сравнения (`docs/KBL.md` 7.1.28, 7.1.35);
* **два бага порядка**: `build_kbl_param()` вызывался *до* `mirror_cmep_scratch_to_arm()`,
  а SPAD32K — это ARM-окно `PA 0..0x3FFF`, поэтому зеркало затирало только что
  построенную запись. Исправлено в стадиях KBL и NSKBL; проверено дампом сразу после
  билдера (`0x1C0=0x60`, `0x1C4=0xFF14`, `ZLB_KBL_PARAM_LOG=1`, `docs/KBL.md` 7.1.32);
* запись `SceKblParam` теперь строится **одинаково** в обоих режимах (раньше
  подстановочный прогон её не строил вовсе), из-за чего исчезло первое расхождение
  данных: гость читает `0xFF14` на `0x40020290` и там, и там (`docs/KBL.md` 7.1.34).

**Исторические исправления этой серии (до нынешней response-timeout модели):**

* **`Sdif::finish_transfer()` была мёртвым кодом** — объявлена (`soc_internal.h:603`) и
  определена (`sdif.cpp:904`), но не вызывалась ниоткуда, поэтому DMA-передача отчитывалась
  только Command Complete. NSKBL превращал это в `0x80320002` и печатал ошибку на UART.
  В этой серии завершение публиковалось дважды: сразу (чтобы пройти диспетчер статуса
  `0x5101EBE4` — бит 0 → `0x80320002`, бит 1 → нормально) и через 16 тиков (драйвер
  подтверждает первое событие записью, чистящей биты 1/4/5, и затем опрашивает
  `0x5101EDE4`). Эффект: ошибка на UART исчезла, arm0 ушёл из терминального цикла
  `0x51000D0C` в драйверный опрос `0x5101EDEA`, чекпойнт `0xA9` сохранён;
* **«намеренно не отвечать» считалось сбоем команды.** CMD5 на eMMC (`sdif.cpp:531`) и
  CMD8 без check-pattern (`sdif.cpp:597`) выставляли `command_ok_ = false`, что поднимало
  `command_error_in_` → `error_status_ |= 0x0001` → `0x80320002` в **статусе запроса**
  `[request+0x28]`, по которому serve-loop выбирает ветку. Тогда был введён отдельный флаг
  `command_no_response_`. Эффект: появлений `0x80320002` в прогоне — **ноль**
  (`ZLB_NSKBL_ERR_TRAP=1`), первый вход диспетчера давал `r3 = 0`. Это подавление
  статуса позднее оказалось причиной нового зависания. Теперь отсутствие ответа
  CMD8/CMD5 завершается настоящим Command Timeout через delayed completion,
  error-summary/IRQ и W1C ACK; гость проходит SD/SDIO probes и выбирает MMC
  (верхний текущий статус).

**Историческая стена этой серии (до исправления CMD8/CMD5 timeout).**
`--stage kbl`, `runm 2000000`: arm0 в плотном опросе `0x5101EDE4`
(511 954 703 инструкции), чекпойнт `0xA9`. Цикл ждёт ненулевого
`[SDIF+0x30]`/`[SDIF+0x32]`; все события доставляются и подтверждаются (трасса
`ZLB_SDIF_TRACE=1`: `0x003A` — биты 1,3,4,5), но драйвер возвращается в опрос. Разбор
serve-loop'а `0x5101EDC8` показал: его возвращаемое значение — **статус запроса
`[r9+0x28]`**, и именно его диспетчер `0x5101F4D0` сравнивает с `0x8032000C`/нулём.
Эта остановка устранена response-timeout моделью. Текущая граница после успешных
чтения/auth/decrypt первого os0-файла — native decompression `0x80560100` →
`0x800F0516`, описанная в верхнем текущем статусе.

Инструменты серии: `ZLB_ARM_PC_LOG`/`ZLB_ARM_PC_REGS` (построчный diff трасс двух
прогонов — именно он нашёл первое расхождение), `ZLB_EXCL_LOG`, `ZLB_NSKBL_ERR_TRAP`
(ловит появление значения в регистре, не зависит от устаревших карт адресов),
`ZLB_NSKBL_POLL_LOG`, `ZLB_EMMC_LOG`, `ZLB_SDIF_TRACE`.

## 5. Как запускать и проверять

```powershell
# сборка
powershell -ExecutionPolicy Bypass -File .\build.ps1

# самотесты
.\build\bin\zlb_tests.exe

# полная проверка (сборка, самотесты, раскладка, план, eMMC, стадии загрузки)
powershell -ExecutionPolicy Bypass -File .\verify.ps1

# цепочка целиком (шаги 1-4 вики) и текущее состояние
.\build\bin\zeliboba.exe -q -ex "boot" -ex "runm 300000" -ex "boot" -ex "gpo" -ex "quit"

# отключить все подстановки
$env:ZLB_NO_SUBSTITUTION=1
```

### macOS / Apple Silicon (проверено 2026-10-01)

Нативная сборка AppleClang + CMake работает для CLI, инструментов, самотестов
и SDL3. SDL3 3.4.16 установлен через Homebrew; окно использует Metal, звук —
48 кГц stereo. Visual Studio Code для сборки не требуется.

```bash
./build-macos.sh
./run-macos.sh --cli
./run-macos.sh
./run-macos.sh --cli -q -ex "boot" -ex "runm 300000" -ex "boot" -ex "gpo" -ex "quit"
```

Раскладка входных файлов — в `README.md`, раздел macOS. `Vita_104_Firmware.zip`
распакован рядом с исходниками; дампы находятся в `../dumps/` и `../ernie-master/`.
Образ создаётся в `build/emmc.img` независимо от имени каталога исходников.

Первый прогон macOS до исправлений CPU: `runm 300000` доходит до **`0xA9`**,
ARM0 опрашивает `0x5101EDE8`; ядро не загружено. Свежая реконструкция eMMC проходит
`emmc_rebuild --verify`: **992 файла, 0 расхождений**, четыре копии SLB2 проверены.
Самотесты: **438 запущено, 0 отказов, 0 нарушений ассертов**. Оба дампа
first_loader (`vita_prototype_bootrom.bin` и `pch-5c-cold_first_loader.bin`)
есть в `../dumps/`; полный прогон использует prototype-дамп.

Независимая проверка `tools/verify_emmc_bytes.py` против исходного ZIP подтверждает
совпадение байтов обоих os0, vs0 и всех четырёх SLB2; FAT-копии совпадают,
`fsck_msdos -n` для os0/vs0 возвращает 0. Сборщик теперь сохраняет исходный SLB2
целиком: прежние отличия в незанятом padding (`0xFF` → 0) устранены. Результат
загрузки до и после этой правки одинаков: **`0xA9`**, ARM0 `0x5101EDE8`.
Старый реконструированный образ сохранён в `build/emmc-before-verbatim.img`.

Журналы проверки: `build/macos-build-script.log`, `build/macos-tests.log`,
`build/macos-emmc-verify.log`, `build/macos-emmc-bytes.json`,
`build/macos-boot.log`, `build/macos-window.log`.

**Правило измерений.** Подстановки цепочки загрузки живут в `Vita::satisfy_arm_boot_pc`
и вызываются только из `Vita::run_slice`. Команда отладчика `run` крутит `step(1)`
и **обходит** их, поэтому замеры KBL/NSKBL надо делать через `runm <срезы>`, а точки
остановки ставить как `bp arm <addr>` (без архитектуры адрес уходит в MeP).

Диагностические переменные: `ZLB_SDIF_TRACE`, `ZLB_BIGMAC_TRACE`, `ZLB_EMMC_LOG`,
`ZLB_BOOTKEY_TRACE` (раскладка констант fitted-загрузчика и три дайджеста
провижининга: digest_info, SHA-256 заголовка, SHA-256 подписываемого блока),
`ZLB_ARM_FAULT_LOG` (отказы ARM в режиме `ZLB_NO_SUBSTITUTION=1` — там хук выходит
раньше своей трассы), `ZLB_SECURE_FAULT_LOG` (отказы ARM с признаком `secure`,
TTBR0/TTBR1 и DFSR/DFAR/IFSR/IFAR; работает и при включённых подстановках),
`ZLB_ARM_TRACE_RING=<pc>` + `ZLB_ARM_TRACE_RING_SIZE=<n>` + `ZLB_ARM_TRACE_RING_DUMPS=<n>`
(кольцо последних инструкций до указанного pc; размер по умолчанию 16384, число дампов —
1, кольцо между дампами сбрасывается, так что `DUMPS=k` сравнивает k-е посещения одного
pc), `ZLB_SKBL_ORDER_LOG=1` (порядок событий SKBL: входы `0x40031CB0`,
`0x4002F070` с `r0`, описание окна `0x40029F34`/`0x4002DBD0` с `r8`/`r9`),
`ZLB_KBL_WFE_TICK=1` (будит кластер таймерным тиком `PPI 29` через `ArmCore::irq_hook`
вместо правки счётчика барьера; период — `ZLB_KBL_WFE_TICK_SLICES=<слайсов>`, по
умолчанию 64. Измерено: тик при любой частоте доводит прогон только до `0x49`/`0x88`,
`docs/KBL.md` 7.1.12), `ZLB_KBL_WFE_PATCH_LIMIT=<n>` (ограничивает число правок счётчика
барьера: без флага — 23 правки и `0xA9`, с 12 — `0xA4`, с 1 — `0x49`; `docs/KBL.md`
7.1.13), `ZLB_ARM_CORE_STAGGER=<n>` (удерживает ядро `i` на `i*n` инструкций —
проверка гипотезы о разбеге ядер: без флага `0x8D`, с разбегом `0x84`, `docs/KBL.md`
7.1.21), `ZLB_ARM_BUDGET=<n>` (блочное планирование: ядру даётся `n` инструкций подряд;
в честном режиме `0x8D` при `n=1` и `0x54` при `n=4/16`, `docs/KBL.md` 7.1.23),

**Пункт 1 цели (зеркало CMeP scratch → ARM PA 0) закрыт.** `mirror_cmep_scratch_to_arm()`
копирует все 32 КиБ (`shared_sram_` → `arm_bus_` VA `0..0x7FFF`) и вызывается на обоих
путях release, без подстановок. Проверено по шине ARM (`map 0x34`): `region=arm_bootrom`,
`page-region arm_bootrom base=0x00000000 size=0x40000`; после полного boot чтение ARM по
PA `0x40000` отдаёт данные загрузчика (`00 B0 00 00 12 70 00 00 …`), то есть диапазон
действительно виден ARM. Ранний диагноз «`arm_bootrom` отдаёт нули» был ошибкой
измерения: дамп шёл по шине CMeP (`active_core()` по умолчанию — MeP). Остаточное
наблюдение: слова по PA `0x100` в конце прогона — `0xFFFFFFFF`, то есть зеркало к этому
моменту перезаписано (нулевой слепок в момент самого зеркала виден в логе; контекст
пишется позже). На поведение SKBL это не влияет.
`ZLB_MEP_PC=addr[,addr…]`, `ZLB_WTRAP=lo-hi`, `ZLB_RTRAP=lo-hi`,
`ZLB_ARM_LOW_MAP=identity|dram|dram-abs|window`, `ZLB_KBL_FAULT_TRACE`,
`ZLB_KBL_PANIC_TRACE`, `ZLB_KBL_TRACE_PC=<hex>`, `ZLB_ARM_TRACE_RANGE=lo-hi`,
`ZLB_NO_SUBSTITUTION=1`.

**Покрытие кода.** `ZLB_ARM_COV=1` (окно по умолчанию — образ NSKBL
`0x51000000+256 КиБ`; `ZLB_ARM_COV_BASE`/`ZLB_ARM_COV_SIZE`/`ZLB_ARM_COV_GRAN`
меняют окно и шаг) и `ZLB_MEP_COV=1` для CMeP. Печатает карту команда
`cov [mep] [start] [bytes]`, сохраняет сырой битмап — `cov save <file> [mep]`;
`python tools/cov_report.py --preset arm|nskbl|mep --bitmap <file>` считает
доли по образам и проверяет список точек входа (`--query имя=адрес`).

## 6. Дальнейшие шаги

1. **Закрыть §3.0**: сделать так, чтобы устройство NSKBL отвечало на управляющий
   запрос — либо найденным состоянием устройства, из которого драйвер сам строит
   запрос `0x514` (диспетчеризуемый), либо реализацией ответа на его
   completion-списке. Обе ветки упираются в одно: устройство по `VA 0x240` —
   единственная структура, которую цепочка загрузки нигде не пишет.
   Проба «дать пул свободных узлов» (`ZLB_NSKBL_DEV=1`, 8 узлов) измерена и оказалась
   **регрессом**: чекпойнт `0xA4` вместо `0xA9`, все ядра в спин-замке `0x510147DC`,
   новых чтений карты нет. Раскладка запроса при этом снята полностью (`+0x08` —
   индекс команды, `+0x0C` — аргумент SDIF, `+0x7C` — таблица ADMA2, `+0x04` бит
   `0x400` — «диспетчеризуемый»), поэтому следующий шаг — не узлы, а **выполненная
   передача**: распознать запрос данных, провести чтение через `Kermit.Sdif0` и
   вернуть узел в `[device+0x2400]` уже завершённым (подробности — `docs/NSKBL.md` §7).
2. После этого — модули ядра: `os0:psp2bootconfig.skprx`, `SceSysStateMgr`,
   `SceKernelBootimage`, затем `vs0` и `shell.self`.
3. **Stage 2**: Venezia MPE (`docs/VENEZIA.md`) и PowerVR SGX543MP4+
   (`docs/GPU.md`) на OpenGL с шейдерами, затем LiveArea. GPU начинается после
   того, как ядро реально грузит модули `os0`.

## 7. Метрики на текущий момент

Актуальные числа этой сессии (2026-10-02) — в разделе «Текущая проверка» выше.
Историческая таблица ниже относится к прогонам **до** запуска модулей os0 и
текущую границу не описывает.

| Метрика | Значение |
|---|---|
| Самотесты | **627 прогонов / 0 падений** (VS2026 Release x64); `verify.ps1` требует ноль падений по умолчанию |
| eMMC | `build/emmc.img` пересобран с нуля: 992 файла, 0 расхождений; MD5 `b3e8f46f5e948bb7dcac06096b0bb014` |
| Цепочка | Syscon → first_loader → second_loader → secure_kernel → ARM KBL → NSKBL → **os0 kernel** одним прогоном, без команд `stage` |
| Стадия | `boot` теперь сам доходит до `kernel-running` (NSKBL `0x51000000`, ядро с `0x000C43B2`) |
| eMMC в прогоне | **10753 чтения, 0 записей**; читается только раздел os0 (`lba` до `72800`), раздел vs0 (LBA ≥ 360448) — ни разу |
| Покрытие | 1M дополнительных слайсов из `s600k` дают **+193** новых сайта ARM (страницы `0x4A2000…0x4AA000`, `0x4F4000`, `0x5B7000`); дальше рост прекращается — система уходит в простой |
| Простой | `s1000k` + 5M слайсов: `t` = **4.6126 s**, все 4 ядра `halted=1 reason=wfe`, eMMC по-прежнему `10753 reads / 0 writes`, консоль пуста |
| Эмулированное время | `s1000k` = 0.769 s (482M инструкций); ≈0.77 µs гостевого времени на слайс, ≈6–7M инструкций/с |
| Ядро | **загружено и исполняется, затем уходит в idle**; граница — до монтирования vs0 и запуска SceShell |

