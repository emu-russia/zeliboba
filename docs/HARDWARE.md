# Карты памяти и регистров

Каждый адрес помечен: **[Ф]** — факт из материалов рабочей папки,
**[Р]** — получено реверсом модулей прошивки, **[М]** — модельное допущение.

## 1. CMeP (F00D, MeP-c5)

### 1.1 Память **[Ф]**

| Диапазон | Размер | Что |
|---|---|---|
| `0x00040000..0x0005FFFF` | 128 КиБ | RAM CMeP; boot backing first_loader (`0x5C000`), staging второй стадии (`0x40000`). На secure-kernel handoff модель переключает окно на backing `0x800000` **[М]**, чтобы исполнялись настоящие uncached cache helpers `0x400B0/0x400CE` **[Р]**; выбирающий hardware register неизвестен |
| `0x00060000` | — | вершина стека: при сбросе `$0` = `0x60000`, `_start` копирует его в `$sp` |
| `0x0005EB00..0x0005EE3C` | 0x33C | `.bss` first_loader, обнуляется в `_start` |
| `0x0005FFE0..0x0005FFFF` | 32 | stub самоуничтожения точки входа |
| `0x00800000..` | 2 МиБ | приватная RAM CMeP (secure kernel) **[М]** |
| `0x1F000000..0x1F03FFFF` | 256 КиБ | загрузочная SRAM, общая с ARM (алиас одного буфера) **[М]** |

### 1.2 Регистры **[Ф]**

| Адрес | Имя | Поведение |
|---|---|---|
| `0xE0000000/04/08/0C` | четыре mailbox CMeP→ARM | канал 0: `1` = успех first_loader, `2` = отказ; secure_kernel использует также `0x101/0x102` и уведомления в старших битах **[Р]**; запись CMeP устанавливает биты, запись ARM подтверждает и сбрасывает выбранные биты **[М]**, см. 1.2.1 |
| `0xE0000010/14/18/1C` | четыре mailbox ARM→CMeP | канал 0: бит 0 = запрос готов; при передаче образа/общего буфера биты 2+ содержат физический адрес **[Р]**; запись ARM устанавливает биты, запись CMeP подтверждает и сбрасывает выбранные биты **[М]** |
| `0xE0000040..0xE000005C` (шаг 4) | W1C-алиасы восьми mailbox | запись маски сбрасывает выбранные биты соответствующего слова `0xE0000000..0xE000001C` с любого порта **[М]**, подтверждено последовательностями очистки **[Р]/[Ф]** |
| `0xE0000020/24`, `0xE0000028/2C`, `0xE0000060/64` | отладочные mailbox | протокол подтверждения `0xFFFFFFFF` |
| `0xE0020000` | флаги CMeP (arm2cry) | — |
| `0xE0020020` | семафор завершения Bignum: движок поднимает по окончании операции, `engine_wait` (`0x4BBB0`) снимает записью | `0x5CF12`, `0x4BBB0` **[Р]** |
| `0xE0030000..0x1C` | `KeyringNewValue[0..7]` | 256 бит материала ключа |
| `0xE0030020` | `KeyringSetValueTrigger` | младшие 13 бит — индекс keyring |
| `0xE0030024` | `KeyringClearFlags` | сброс флагов; младшие 13 бит — индекс, старшие — маска |
| `0xE0030028` | `KeyringQueryFlags` запрос | пишем индекс (например `0x501`) |
| `0xE003002C` | `KeyringQueryFlags` ответ | `(value & 3) == 3` — keyring готов |
| `0xE0040108` / `0xE0040400` / `0xE0040508` | Bignum: основание / модуль / результат, по 64 слова | `0x5CE0C`/`0x4BA52`, `0x5CE28`/`0x4BA6E`, `0x5CECA`/`0x4BB52` **[Р]** |
| `0xE0040800` | Bignum: управление `0x91000000 \| слов_модуля<<18 \| слов_экспоненты<<9` | `0x5CE5C`, `0x4BAA0` **[Р]** |
| `0xE0040804` | Bignum: статус — бит31 занято, бит25 ошибка, бит26 готов/простой, бит27 ждёт слово экспоненты, биты16..23 длина результата | `0x5CE68..0x5CE8E`, `0x4BAC2..0x4BB3E` **[Р]** |
| `0xE0040808` | Bignum: очередное слово экспоненты (big-endian, от старшего) | `0x5CE50`, `0x4BAF6` **[Р]** |
| `0xE0050000/04/08/0C/10/14/1C/24` | Bigmac: аргументы, функция, старт, статус | `+0x0C` — код операции, `+0x1C=1` — старт, `+0x24` — статус (бит 0 = занят) |
| `0xE005003C` | статус исключения Bigmac | читается на пути отказа |
| `0xE0064060` | состояние SC/JIG | — |
| `0xE0070000` | `EmmcCryptoToggle` | `1` — включить eMMC-крипто |
| `0xE0070008` | индексы ключей eMMC | `0x020E020F` |
| `0xE0010000/04` | системная конфигурация CMeP (`CMeP.SysCtl`) | проверка из реверса `0x40DB6`: `(+0x04 & 5) == 5`, `(+0x04 & 8) == 0` **[Р]**; значение, которое выдаёт модель (`0x00000005`), — **допущение**: отдельные биты ни в одном материале рабочей папки не названы |
| `0xE0062020` | страп/JIG, 32 байта | бит 0 = `'A'`(65)/`'!'`(33) для баннера first_loader (`0x5C116`); вторая стадия требует `(значение & 7) == 0` (`0x40DB6`) **[Р]** |
| `0xE0062120..0xE006213F` | readable identity slot `0x509` | native `0x80571A`: `0xE0058000 + (0x509 << 5)` **[Р]**; public PCH-1001 prefix `00 00 00 01 01 04 00 10`, остальное нули/недоступно **[М]**, не console dump |
| `0xE0062260..0xE006227F` | DRAM capacity slot `0x513` | kprx `0x80E59E..0x80E5D8` читает первые 4 bytes как LE aperture size **[Р]**; `0x20000000` соответствует уже заданной board DRAM и KBLparam **[М]**, остальное unavailable/zero; native bounds checks сохранены |
| `0xE6008000 + i*8` | команда слота `i` неотождествлённого блока CMeP | `0x482CE` **[Р]**, сам блок не отождествлён **[М]** |
| `0xE6008020` | статус: 4 бита на слот | `0x482E6` **[Р]** |
| `0xE6008100 + i*8` / `0xE6008120 + i*8` | 16-битное значение / «записать байт, ответ в битах 8..15» | `0x48310`, `0x48370` **[Р]**; **семантика ответов — допущение**, подогнанное под три проверки загрузчика (`cmep_internal.h`) |
| `0xE6008180` | строб (записать 1, ждать 1) | `0x48422` **[Р]** |
| `0xE20A0000..0xE20A0FFF` | GPIO0, общий объект CMeP/ARM | `+00` — направление, `+04` — вход, `+08/+0C` — set/clear отдельного output latch `+34`; native Syscon использует output3/input4 и IRQ248/sub4 **[Р]**, ограниченная модель описана в §1.2.3 |
| `0xE0100000` | контроллер хранения CMeP (не отождествлён): второй загрузчик обходит два дескриптора в `0x561A4` (GPIO и этот) и инициализирует регистры `+0x00`, `+0x14..+0x2C`, `+0x38..+0x48` (`0x49568..0x495E0`) | `CMeP.Storage` **[Р]** |
| `0x5FFC0000` | командный блок «SCE»: загрузчик копирует туда 48-байтовый запрос и читает ответ оттуда же, проверяя магию `SCE\0`, слово 1 == 3 и бит `0x40` байта 8 (`0x40B10`, `0x4A774`) | `CMeP.SceBlock` (эхо) **[Р]** |
| `0xE3100000..0xE311FFFF` | окно SC на стороне CMeP (**128 КиБ**, `cmep::kScWindowSize`): вторая стадия трогает `0xE31040A4`, `0xE3105000` (`0x48794`, `0x48848`) и `0xE3110C00` | `CMeP.SecureCtl` **[Р]** |
| `0xE3100000+` | окно SC (syscon) | `0xE3100124`, `0xE31010A0/A4`, `0xE3101100`, `0xE3101190`, `0xE31020A0/A4`, `0xE3102100`, `0xE3103040/50` |
| `0xE0B00000 + (cmd<<16)` | окно передачи SC-команды | дескриптор 28 байт в `0x5EE20` |
| `0xE0BF0000 + (cmd<<16)` | окно ответа SC | — |
| `0x30000118`, `0x30000208` | регистры SoC в пути bit5 | — |
| сопроцессор `0x400..0x405` | задержки (`stcb`/`ldcb`) | 16-битный reload big-endian в `0x400/0x401`, `0x402` бит 0 = пуск, `0x404` бит 0 = «счёт дошёл до нуля» (см. 1.4) |
| `0xE0A00000` | **SPI0 = линия syscon** | вторая стадия CMeP реально общается с Ernie **здесь**, а не через окно SC (см. 1.5) |
| `0xE0A10000`, `0xE0A20000` | SPI1 (motion), SPI2 (OLED/LCD) | та же карта регистров |

### 1.2.1 Общие mailbox: порты, подтверждения и IRQ

**Свидетельства прошивки [Р].** ARM и CMeP читают одни и те же слова, но
используют их с противоположных сторон. В 1.04 CMeP подтверждает входящую
команду записью `0xFFFFFFFF` в `0xE0000010` (`0x8003FC`, `0x80042E`,
`0x800AF2`). Нативный ARM Smsched подтверждает статусы `0x101/0x102`
записью этих значений в `0xE0000000`, а общий буфер отправляет двумя
записями в `0xE0000010`: сначала физический адрес, затем отдельную `1`
(`0x0051E3C2`, `0x0051E3C4` в наблюдаемой загрузке). Следовательно, вторая
запись должна сохранять адрес, а подтверждение — сбрасывать лишь выбранные
биты. Эти же операции есть в `set_kernel_enp` опубликованного
[xyzz/f00d smsched.c](https://github.com/xyzz/f00d/blob/master/smsched.c) **[Ф]**.

Очистка собственных исходящих слов также наблюдается отдельно: CMeP пишет
`0xFFFFFFFF` в `0xE0000044/48/4C` (`0x8005FE..0x80061E`,
`0x8023F8..0x802416`) **[Р]**; ARM использует `0xE0000054/58/5C` перед
новым запросом в `smsched_load_task` того же проекта **[Ф]**.

**Реализованная модель [М].** `CMeP.Mailbox` и отдельный ARM-порт имеют
общее состояние. На порту отправителя запись работает как W1S (единицы
устанавливают биты); на порту получателя — как W1C (единицы сбрасывают
биты). Нули сохраняют состояние. Алиасы `+0x40` всегда работают как W1C,
в том числе при очистке собственного исходящего слова. Байтовые и
полусловные обращения сохраняют соседние байты. Прямые host-методы
`set_arm_to_cmep_command` и `set_cmep_status` присваивают значение целиком
через `poke`; они обслуживают существующие подстановки загрузочной модели.

| Направление | Слова каналов 0..3 | Подключение IRQ в модели | Условие активного уровня |
|---|---|---|---|
| ARM→CMeP | `0xE0000010/14/18/1C` | CMeP INTC, источники `8/9/10/11` | канал 0: установлен бит 0; каналы 1..3: слово ненулевое |
| CMeP→ARM | `0xE0000000/04/08/0C` | ARM GIC, ID `200/201/202/203` | слово ненулевое, включая уведомление `0x10000` |

Для CMeP прошивка содержит отдельный вектор источника 8, ведущий через
`0x80035C` в обработчик mailbox `0x800A42`, и регистрирует обработчики
источников 9..11 в `0x8023E0..0x8023F4` **[Р]**. Нативный ARM Smsched
регистрирует IRQ 200..203; `init_int` в xyzz/f00d подтверждает эти ID
**[Р]/[Ф]**. Связь четырех периферийных каналов с этими линиями и условия
уровня в таблице — реализованное поведение модели **[М]**, основанное на
этих обработчиках и протоколе запросов; отдельная электрическая
спецификация mailbox в рабочей папке пока отсутствует. Для канала 0
ARM→CMeP запись одного адреса не вызывает IRQ до записи бита 0. В обратную
сторону нельзя ограничиваться битом 0: CMeP публикует `0x10000` в
`0x8009CE` **[Р]**.

Уровни обновляются после установки и подтверждения битов, записи в
алиас, прямого host-присваивания и сброса устройства. Частичное
подтверждение сохраняет IRQ, если остается условие активности; сброс
очищает слова и снимает все уровни. Подключение callback передает уже
ожидающие уровни. Mailbox не создает ответ за прошивку: обработчик CPU
получает прерывание и сам читает, подтверждает и обслуживает запрос.
В текущем ARM GIC реализованы четыре CPU-интерфейса, banked private interrupts
и Secure/Nonsecure register views. Native Smsched задаёт Group0, target `8`
(ARM3) и FIQEn. SPI reset соответствует Cortex-A9 ICDICFR `0x55555555`:
level-sensitive trigger и fixed handling-model bit; SGI/PPI имеют свои reset
значения. Поэтому polling ACK до enable снимает pending mailbox event.
Следующий настоящий status вызывает FIQ200 на ARM3, затем Nonsecure SGI4 self
(`SGIR = 0x02008004`), IAR/EOI token `0xC04` (ID4, source3).
Нативный callback пишет completion в `0x5113BAC0`; IRQ/FIQ после ACK сняты.
Свидетельства: `build/goal-native-fiq-level-full.log`,
`build/goal-sgi-completion-capture.log`; cross-bus regression в `test_soc.cpp`.
Проверки через оба Bus-порта находятся в `tests/test_cmep.cpp`
(`cmep_mailbox_native_shared_buffer_handshake`,
`cmep_mailbox_endpoints_ack_only_selected_bits`,
`cmep_mailbox_doorbell_and_status_irq_levels`,
`cmep_mailbox_clear_aliases_and_irq_lifecycle`).

### 1.2.2 CMeP INTC на control bus и выбор вектора

**Архитектурная основа [Ф].** Toshiba *MeP Core (MeP-c4) User's Manual
(Architecture)*, `MEPUM05005-E22`, §3.4.2 (печатная с. 57), и *Control Bus
Unit User's Manual*, `MEPUM03015-E11`, §2.2 (с. 6), описывают отдельное
пространство с адресацией по 32-битным словам. `LDCB/STCB` передают полное
32-битное значение. INTC занимает CB `0..7` (Architecture §5.1,
с. 234–244):

| CB | Регистр | Смысл |
|---|---|---|
| `0` | IVR | номер принятого источника ICN `[7:3]`, маска IML `[11:8]`, его уровень ILV `[15:12]`; запись меняет IML |
| `1` | ISR | ожидающие запросы; уровень отражает входную линию, защёлка фронта очищается записью нуля (W0C) |
| `2` | IER | бит разрешения для каждого источника |
| `3` | IET | выбор фронта (`1`) или уровня (`0`) |
| `4..7` | ILR0..3 | приоритет каждого источника, по 4 бита |

INTC запрашивает IRQ, когда источник ожидает, разрешен в IER и его
приоритет строго выше IML; больший приоритет выигрывает, при равенстве
выбирается больший номер источника. Такой запрос выводит CPU из
`SLEEP/HALT` даже при сброшенных PSW.IEC/HIE, но вход в обработчик требует
обоих разрешений (Architecture §5.2–5.3, с. 245–248). При входе CPU
сохраняет EPC и состояние IEC; `RETI` восстанавливает IEC из IEP и
возвращается по EPC (§3.6.5).

**Настройка прошивки 1.04 [Р].** `0x8002C0..0x8002FA` записывает
`IET=0`, `ILR0=0x07777777`, `ILR1=0x0000777F`, `ILR2/3=0`,
`IVR=0x600` (IML=6), `IER=0x100` (источник 8) и поднимает PSW.HIE.
Источник 8 имеет приоритет 15. Регистрация обработчика в `0x802572`
затем меняет соответствующий бит IER через 32-битный `1 << source`.

**Векторная база Vita [Ф]/[Р]/[М].** Общая архитектура Toshiba при
CFG.EVM=0 использует базу 0; при EVM=1 выбирает RAM-банк через EVA/IVA,
а IVM=1 добавляет `4 * source` к базе IRQ (Architecture §3.6.2,
с. 115–116). [Vita wiki Cmep/Reset](https://wiki.henkaku.xyz/index.php?title=Cmep&oldid=20184#Reset)
сообщает об отличающейся базе `0x40000` при EVM=0;
[Secure Kernel/Entrypoint](https://wiki.henkaku.xyz/index.php?title=Secure_Kernel&oldid=21795#Entrypoint)
описывает ее перенос на `0x800000` для secure_kernel **[Ф]**. В 1.04
`0x800140..0x80014A` действительно очищает EVM и устанавливает IVM **[Р]**.

Модель задает базу EVM=0 через `set_boot_vector_base`: `0x40000` при
сбросе first/second loader и `0x800000` на переходе в secure_kernel.
Регистр реального remap пока не определен; это явная подстановка платы
на границе стадий **[М]**. Общий CPU сохраняет архитектурную базу 0 по
умолчанию и обычный выбор EVA/IVA при EVM=1. Поэтому при CFG=`8`
источник 8 выбирает подлинный слот `0x800000 + 0x30 + 8*4 = 0x800050`:
прошивка сама переходит `0x800050 → 0x80035C → 0x800A42` и обслуживает
mailbox. Адрес обработчика и ответ команды не подставляются CPU.

**SWI [Ф]/[Р].** Architecture §3.6.6/§7.13: `SWI n` или `STC EXC` ставит
SIPn (`EXC[4+n]`). На следующей границе инструкции IEC и соответствующий
SIE (`PSW[4+n]`) разрешают вход; SWI имеет приоритет над hardware INT (table17).
CPU сохраняет next PC в EPC, IEC/UMC в IEP/UMP, очищает текущие IEC/UMC,
задаёт EXC code5 и входит в exception base+`0x14`. SIP очищает сам handler.
Модель теперь доставляет SWI по этим правилам, с обычным EVA/board remap.
В native kprx handler `0x800014 → 0x80037A` действительно регистрирует
IRQ9 callback `0x80E55E`, syscall возвращает `1`, модуль входит в work loop.
Размещение SWI в последних двух repeat slots запрещено manual и не заявляется
как поддержанное. Новые tests проверяют ACK/RETI, masking, priority и допустимый repeat.

### 1.2.3 GPIO0: направление, уровни и native IRQ

**Карта [Р]/[Ф].** Lowio 1.04 `PortRead` (`0x81002780..0x810027BA`)
читает `+34`, если выбранный бит направления равен1, и `+04`, если равен0.
Направление не является output latch. Адреса подтверждает аппаратно проверенный
[vita-libbaremetal gpio.c](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/gpio.c).
Регистры ниже находятся относительно `0xE20A0000`:

| Смещение | Назначение | Реализованное поведение |
|---|---|---|
| `+00` | direction: hardware1=output,0=input | отдельное слово направления; API Lowio инвертирует свой аргумент **[Р]** |
| `+04` | sampled input | outputs читаются из latch/direction, inputs — из board peer; чтение не потребляет состояние **[М]** |
| `+08/+0C` | output SET/CLEAR | только записанные единичные биты устанавливают/сбрасывают latch; соседние byte lanes сохраняются |
| `+10` | неприсвоенный boot snapshot | хранится без выдуманной семантики направления или ready **[М]** |
| `+14/+18` | по2 бита interrupt mode на pin | mode3=falling edge для input4; остальные pin/mode пока не создают событий **[Ф]/[М]** |
| `+1C..+2C`, шаг4 | пять active-high masks | бит1 блокирует соответствующий pin; reset=`FFFFFFFF` **[М]** |
| `+34` | output-latch readback | nondestructive; checkpoint16..23 и диагностический `gpo` читают настоящий latch |
| `+38..+48`, шаг4 | пять pending status | hardware-owned W1C, только supplied byte lanes; reset0 **[М]** |

Определения [gpio.h](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/include/gpio.h)
дают high-level0,low-level1,rising2,falling3 и Syscon output3/input4 **[Ф]**.
Модель поддерживает только фактически достигнутый input4/falling3 путь.
Каждый parent GIC`248+gate` имеет уровень `(pending & ~mask) !=0`.
Сохранение masked edge, fanout в пять status и all-masked reset — явные
политики модели **[М]**, не измеренные электрические reset значения. В reached
native setup открыт только gate0. Изменение mode/mask/direction само не
создает фронта; reads/peek/W1C не меняют физический input. Неизвестные слова
остаются unavailable (`FF`); reset не сохраняет прежние stray writes.

Development JIG отсутствует по умолчанию. Его отдельный opt-in API поднимает
input4 и освобождает его по настоящему driven output3 rise, а не по чтению
**[М]**. На уже моделируемой границе secure-kernel handoff начинается native
phase: JIG отключается и повторно не включается до reset. Реальный register
выбора такого board phase неизвестен. Phase управляет только development JIG,
а не доступностью настоящего SPI reply: второй загрузчик тоже ждёт GPIO pending.

`tests/test_gpio.cpp` проверяет неизменённый Lowio PortRead, checkpoint latch,
byte lanes, edge/mask/W1C, unsupported modes/reset, JIG phase и общий объект
через два Bus с настоящим GIC248 IAR/EOI. Это проверка register/IRQ contract;
завершение полного native Syscon/sub4 callback пока отдельно не подтверждено.

### 1.3 Header образа второй стадии (`second_loader.enp`) **[Ф]**

| Смещение | Поле | Значение для 1.04 |
|---|---|---|
| `+0x00` | magic | `0x64B2C8E5` |
| `+0x04` | size | `0x2C0` |
| `+0x08` | offset | `0x10` |
| `+0x0C` | length | `0` |
| `+0x10` | field | `0x16600` |
| `+0x16` | u16 | `0` |

### 1.4 Таймер задержек на control bus (`stcb`/`ldcb`) **[Р]**

Оба загрузчика **умеют** им пользоваться (`first_loader` 0x5E660, `second_loader` 0x45500),
но на проверенном пути загрузки его трогает только вторая стадия: покрытие CMeP
(`ZLB_MEP_COV=1`) показывает `0x45500`/`0x45516` исполненными, а `0x5E660`/`0x5E686`
— **нет** (первый загрузчик идёт веткой mailbox bit0 и до своей задержки не
доходит).

| Адрес | Смысл |
|---|---|
| `0x400` (старший байт), `0x401` (младший) | 16-битный reload, big-endian: первая стадия пишет младший байт в `0x401` (`stcb $1,0x401`), вторая — старший в `0x400` (`mov $3,1; stcb $3,0x400`) |
| `0x402` | управление, бит 0 = пуск (пишут `1`) |
| `0x404` | статус, **бит 0 = «счёт дошёл до нуля»**; запись нуля сбрасывает защёлку |

Ключевой факт: бит 0 — это не «занято», а «готово». Оба опроса ждут его установки:
`0x5E686: ldcb $0,0x404 / and $0,$2 / beqz $0,0x5E686` и
`0x45516: ldcb $3,0x404 / and3 $3,$3,1 / bnez $3,0x45520`.

### 1.5 SPI передача пакетов syscon **[Р]**

Регистры (VitaDevWiki «SPI Registers», подтверждено обоими драйверами — `second_loader` 0x436E4 и `syscon.elf` 0x8100003C):

| Смещение | Имя | Поведение |
|---|---|---|
| `0x00` | `SPI_RXFIFO` | чтение: вытолкнуть 16-битное слово (младший байт первым) |
| `0x04` | `SPI_TXFIFO` | запись: положить 16-битное слово |
| `0x08` | `SPI_CTL` | конфигурация (оба драйвера пишут 0 перед стартом) |
| `0x0C` | `SPI_INTCTL` | управление прерываниями |
| `0x10` | `SPI_STATUS` | запись бита 0 = старт, чтение бита 0 = «занято» |
| `0x24` | `SPI_INT_STATUS` | бит 9 = «RX FIFO не пуст», запись 1 сбрасывает (драйверы пишут `0x600`) |
| `0x28` | `SPI_RXFIFO_STATUS` | сколько **байт** можно прочитать |
| `0x2C` | `SPI_TXFIFO_STATUS` | сколько байт ещё в TX FIFO |

Формат пакета (проверен по логу загрузки на VitaDevWiki и по коду самой второй стадии `0x439CC..0x43A32`):

```
запрос:   [cmd_lo][cmd_hi][len][payload (len-1)][checksum]
ответ:    [cmd_lo][cmd_hi][len][flags][payload (len-2)][checksum][pad]
checksum  = ~(сумма всех байт перед ним) & 0xFF;  всего байт = len + 3
```

Длина в запросе = payload+1, в ответе = payload+2 (в ответе есть байт flags). Нечётный
пакет добивается одним байтом, чтобы линия осталась 16-битной (в логе это «unknown» байт).
Пример: запрос CMD 0x0001 без payload — это ровно `01 00 01 FD`, а ответ на CMD 0x0005 —
`04 00 06 00 00 60 40 00 55`.

#### 1.5.1 Native SPI0 response-ready через GPIO0

**Порядок прошивки [Р].** Syscon 1.04 стартует SPI0 (`+10=1`) в
`0x8100011E`, затем поднимает output3 в `0x8100012A`; зарегистрирован
GPIO248/sub4 handler `0x81000160`. Baseline579 действительно достиг
transfer32 с10 ответными bytes и output3 rise, но sub4 callback еще не был
доставлен. [Аппаратно проверенный syscon.c](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/syscon.c)
тоже ожидает GPIO interrupt pending, подтверждает его, читает RX, останавливает
SPI и опускает output3 **[Ф]**; он не устанавливает длительность физического ready.

Тот же контракт нужен до secure-kernel handoff **[Р]**. Second loader
`0x43896/0x438A0/0x438AA/0x438BA` задаёт output3, input4, mode3 и unmask4.
Receive loop `0x43B62` вызывает QueryIntr `0x49790`; `0x43B66` повторяет его,
пока bit4 отсутствует в OR пяти `status & enableShadow`. Этот query не читает
физический input `+04` или output `+34`. Затем `0x43B92` вызывает ACK
`0x49806`, который пишет bit4 в пять W1C status, а native code drains RX,
останавливает SPI в `0x43C62` и опускает output3 в `0x43C72`.
Fresh599 regression действительно остановился в `0x497FC`: валидные4 request
и10 reply bytes, RX10, mode300, mask0=`FFFFFFEF`, output3 high, все pending0.
Handoff-only gate ошибочно исключал нужный ранний response edge; это не запрос
development JIG и не причина возвращать старое read-consuming поведение.

**Реализованная wire policy [М].** Только свежий полный request/reply в SPI0
`CTL=0`, с допустимой длиной/checksum и без старого недочитанного RX,
может сопоставляться с новым driven output3 rise после start при low3.
До валидного frame SPI не предоставляет pin4 peer. После него линия становится
idle-high; требуются input4/mode3 и немаскированный pin4 gate0, независимо от
JIG phase. Допустимый error packet
означает transport-ready; result bytes не превращаются в service success.
После одного PERIPHCLK tick board input API дает input4 falling pulse и
возврат idle-high в пределах этого tick. Latency, idle-high и ширина pulse —
модельные допущения, не электрические измерения.

Выбор pulse вместо persistent-low опирается на точные native bytes **[Р]**:
mode3 setter очищает shadow+28 bit4; parent `0x810022A4..0x810022CC` выбирает
`((sample04 XOR shadow28) & ~shadow24) & gateShadow`. Samplelow исключает
sub4, samplehigh дает candidate`0x10` и ветку `0x810022D6`. Сам pending
получается от физического falling edge в GPIO; GIC248 выбирает core по guest
registers. Прямого subcallback, event, result или принудительного wake нет.
Stop, RX drain, output3 low, потеря qualifier и reset отменяют scheduled pulse;
одна generation дает один edge. Reads/W1C не двигают время и не дают нового edge.
Kermit tick вызывает SPI0 один раз, без повторного tick через DeviceMirror.

`tests/test_spi_syscon_ready.cpp` проверяет start-before-rise/tick порядок,
cancellation/reset, invalid/absent/error replies, stale RX, настоящее
GPIO248→GIC и неизменённый native candidate block: high после pulse проходит,
persistent-low control отвергается. Candidate fixture доказывает eligibility,
не исполнение полного callback. Свидетельства и границы:
`build/goal-native-gpio-spi0-ready-implementation.md`.

#### 1.5.2 Ограниченный SPI2 OLED stream

Native 1.04 OLED стартует пустой SPI2 с точным `CTL=0x30001`, затем пишет
low16 слова; read helper останавливает engine до чтения captured RX **[Р]**.
Только этот port/mode включает stream. Empty start ничего не clock-ит;
prequeued слова и последующие TX writes потребляются синхронно. Stop0
сохраняет RX, смена CTL disarm-ит будущие clocks, новый start нужен для rearm.
Остальные ports/modes сохраняют whole-frame callback.

Каждое потреблённое low16 слово получает16 high input bits (`FFFF`) **[М]**.
Это явно выбранный undriven pulled-high input, не восстановленный Vita pull,
подключённая OLED-панель или supplier ID. Модель не отвечает на конкретные
panel commands и не подменяет ready/script/framebuffer. FIFO counts используют
существующие byte queues **[М]**; native OLED пока доказал только empty/nonempty
и drained-zero. Rate, capacity, overflow и clock divisor не моделируются.
Используется прежний RX-not-empty latch bit9/mask/W1C; native INTCTL3 его
не открывает. Первые64 bytes сохраняются для диагностики, это не FIFO capacity.

Шесть `tests/test_spi_oled.cpp` выполняют неизменённый A1 helper/worker:
девять TX words (`030A 0804 2010 0040`, затем пять `0000`),18 sampled bytes,
пять payload`FF`, сохранённый stack и native close. Worker отвергает supplier
`FFFF`, публикует ready2 и сохраняет API error`803F0A03`; WaitReady возвращает0
для завершённого отказа. Исходные unbound imports fixture сохраняются: это не
проверка native Lowio clock/GPIO или реально подключенной панели. Дополнительно
проверены stop/RX retention, reset, CTL/rearm и legacy framed modes.
Подробности: `build/goal-spi-oled-stream-implementation.md`.

Последний integrated build прошёл617 tests/0 failures. Native Syscon
checksum SIMD теперь также работает; новый ранний NVS profile и сохранение native
allocated stacks позволяют гостю создать настоящий logo buffer. Native IFTU
frame boundary/IRQ204/ACK/rearm/replay публикуют его; actual SDL capture показывает
белый PlayStation logo на чёрном фоне. Это дополнительно подтверждено ordinary
boot captures, а не только unit tests. Физический panel ID, точный fade/blending
и полный hardware protocol остаются неизвестны (`FIRMWARE_DISPLAY_104.md`).

## 2. ARM Cortex-A9 («Kermit»)

| Диапазон | Размер | Что | Источник |
|---|---|---|---|
| `0x00000000..0x0003FFFF` | 256 КиБ | низкое окно ARM: алиас power scratchpad / общей загрузочной SRAM (`board::kArmBootWindowSize`); именно сюда модель зеркалит scratch CMeP | **[М]** |
| `0x00040000..0x0005FFFF` | 128 КиБ | зеркало CMeP SRAM `0x00800000` («MeP boot») | **[М]** |
| `0x1F000000..0x1F03FFFF` | 256 КиБ | on-chip SRAM, общая с CMeP | **[М]** |
| `0x1C000000..0x1C1FFFFF` | 2 МиБ | Scratchpad SRAM | **[М]** |
| `0x1D000000` | 4 КиБ | аппаратный «/dev/null» (чтение 0, запись игнорируется) | **[М]** |
| `0x40000000..0x5FFFFFFF` | 512 МиБ | окно DRAM, видимое и ARM, и CMeP (KBL линкуется на `0x40020000`) | **[М]**, адреса подтверждены трассировкой |
| `0x80000000..0x83FFFFFF` | 64 МиБ | окно основной DRAM | **[М]** |
| `0x1A000000..0x1A001FFF` | 8 КиБ | MPCore: SCU, GIC, GT, PT | **[Ф]/[Р]** — PERIPHBASE из [Physical Memory](https://wiki.henkaku.xyz/vita/Physical_Memory), подтверждается native IntrMgr: CPU interface `VA 0x28022100 → PA 0x1A000100`, distributor `VA 0x28023000 → PA 0x1A001000` |
| `0x1A002000..0x1A002FFF` | 4 КиБ | PL310 L2 cache controller | **[Ф]/[Р]** — тот же источник; KBL непосредственно пишет `0x1A002064` |
| `0xE0500000..0xE0500FFF`, `0xE0510000..0xE0510FFF` | по 4 КиБ | I2C0/1, IRQ142/143 | **[Р]** — native Lowio maps; реализован ограниченный idle/reset subset |
| `0xE5020000..0xE5022FFF` | 12 КиБ | IFTU0 A/B и shared control, IRQ204/205 | **[Р]/[М]** — native 1.04 Lowio table; guest-RAM scanout и ограниченная DSI-frame turnover модель реализованы |
| `0xE5050000..0xE5050FFF` | 4 КиБ | DSI0, IRQ213/sub1 vblank | **[Р]** — native 1.04 Lowio/Display; timing contract восстановлен |

Старое размещение MPCore на `0x1E000000` было подстановкой. Кроме того, прежнее
окно L2CC ошибочно занимало все `0x1A000000..0x1A00FFFF` и перехватывало
настоящие обращения к GIC. Native IntrMgr `0x003BE2AC` записывает PMR по
`PA 0x1A000104`, а firmware range checks допускают IRQ `0..255`.
Это подтверждает необходимость GIC на 256 источников, включая mailbox `200..203`.
Регистр ремапа CMeP из §1.2.2 этим не определяется.

Native framebuffer bank+0 содержит PA, +`0x40` format `0x10`, +`0x44/+0x48`
width/height, +`0x4C` blank, +`0x54` padding bytes. Bank+4 **не stride**.
Ordinary mode pair01, +50=1/+58=108/+180=1 и enabled bus допускают один
future DSI-frame turnover после full-word +180=1; hardware-owned current bit1
меняется перед physical IRQ204/205. Raw plane+40 остаётся0: внутренний latch
подтверждается настоящим zero ACK, encoding status не восстановлена. Это
ограниченные model choices, независимо от descriptor pixels и guest pending.
Полный контракт, подтверждённые stores и ограничения IFTU/DSI —
[FIRMWARE_DISPLAY_104.md](FIRMWARE_DISPLAY_104.md). Старый `0xE2100000` display
controller — синтетический fixture, не регистровая карта настоящего driver.

**Консоль Kermit** (подтверждено кодом самого `kernel_boot_loader`, 0x4003BC88):

| Адрес | Что |
|---|---|
| `0xE2030000`, `0xE2040000` | два консольных блока (таблица баз у KBL — `0x4005C058`) |
| `+0x28` | статус: бит 8 = передатчик свободен (KBL ждёт его перед записью) |
| `+0x70` | данные консоли: запись = передать байт, чтение = принять |

Вывод (а не метка в коде): `0xE2040000` — второй консольный блок, а не «SDIF0
alias»: KBL пишет туда те же символы, что и в `0xE2030000` (одна и та же строка
уходит в оба блока), а фактическое использование подтверждается `kermit.cpp`
(второй UART). Пометка `ASSUMPTION` в `soc_internal.h`/`kermit.cpp` — устаревшая.

Периферия размещается по адресам, подтверждаемым реверсом модулей ядра
(`intrmgr`, `systimer`, `sdif`, `dmacmgr`, `uart`/`lowio`, `display`/`oled`); где
подтвердить не удалось, адрес помечен в коде как допущение и вынесен в константу.

### 2.1 Native I2C idle/reset

Обычный прогон 548 останавливается в настоящем Lowio loop `0x005AB1DA..1DE`:
I2C0 mapping `VA 0x280BF000 → PA 0xE0500000`, I2C1 `0x280C0000 → 0xE0510000`.
Прошивка пишет `+2C=0x0100F70F`, `+08/+0C=1`, `+14=7`, ждёт `+1C=0`,
подтверждает `+28` чтением и записью того же значения, затем пишет
`+2C=0x01000000`, `+18=5`. Native handler также использует echo-W1C `+28`.

Независимые primary implementations
[vita-libbaremetal](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/i2c.c)
и [hardware-tested Linux driver](https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/drivers/i2c/busses/i2c-vita.c)
подтверждают reset и назначение `+1C` как Busy. `+00/+04` — TX/RX FIFO,
`+10` — slave address, `+14` — command/length, `+18` — speed,
`+28/+2C` — IRQ status/control. Апертуры и IRQ также присутствуют в
[native device tree](https://github.com/incognitojam/linux_vita/blob/vita-port-6.12.0/arch/arm/boot/dts/vita.dtsi).

`src/hw/soc/i2c.cpp` реализует **только exact reset command 7**, idle,
сохранение control words, byte lanes и W1C. Синхронное завершение reset —
выбор модели **[М]**, latency не восстановлена. Любая иная команда остаётся
явно unsupported/busy до reset; FIFO data, slave ACK/NACK и completion IRQ
не подставляются. `+08/+0C` и отдельные mask bits `+2C` остаются opaque.
Шесть focused tests исполняют original Lowio bytes на обоих портах,
включая восстановление из busy. Это не доказательство полной I2C передачи.
Evidence: `build/goal-native-lowio-i2c-evidence.md`,
`build/goal-native-i2c-isolated/test_i2c.log`.

### 2.2 SceEmcTop и CDRAM

FW1.04 SceDriverTzs maps Secure `0xE8200000/0x1000`, registers SMC117/118/119,
and submits command payload at `+0x28`, launch/busy at `+0x24` **[Р]**.
The model accepts only captured payloads `0xE0000`, `0x40400`, `0x20000`, `0x31`
with launch1, and `0x200000` with launch3. Busy bit0 clears after one peripheral
tick **[М]**; mode request `0x10` produces hardware acknowledgement `0x20`.
Undefined command/flags stay unsupported/busy until explicit control0 **[М]**.
Configuration words remain opaque. Calibration status `+0x240`/W1C `+0x244`
does not produce a fabricated IRQ34 event; its source is still unknown.

The ordinary 579-test binary completes **all six commands** after the native
WT7 delay interrupt. Actual SMC117 returns zero. Electrical calibration and a
native CDRAM memory test remain unproved; no calibration IRQ34 is generated.
Exact protocol and assumptions: `build/goal-native-e820-emctop-evidence.md`.

Independent 128 MiB RAM now backs ARM physical `0x20000000..0x27FFFFFF` **[Ф]/[М]**,
shared with GPU/IFTU through the ARM bus. It is separate from `0x1C000000` SRAM
and main DRAM; there is no guessed CMeP alias. Existing zero-on-reset RAM policy
applies; electrical gating/retention are not modeled.
[Hardware-tested framebuffer code](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/display.c)
uses `0x20000000`; the [physical memory map](https://www.psdevwiki.com/vita/Memory_Mapping)
specifies the aperture. [CDRAM enable](https://github.com/xerpi/vita-libbaremetal/blob/master/libbaremetal/src/cdram.c)
uses the same SMC117 as the native Lowio callback.

### 2.3 LT5 / WT7 timers

The reached FW1.04 time provider uses LT5 `0xE20B6000/0x1000`, IRQ141:
64-bit current `+00/+04`, deadline `+08/+0C`, W1C status `+18`, config `+1C`
**[Р]**. KBL programs `2F345008`, clears current, sets deadline to all ones,
then starts with `2F34500D`. ThreadMgr reads raw low/high counts and ACKs2.
Source3 at48MHz / (47+1) supplies one count per microsecond **[Р]/[М]**.

Secure IntrMgr uses WT7 `0xE20BE000/0x1000`, IRQ135: 32-bit deadline `+00`,
current `+04`, config `+08`, W1C status `+14` **[Р]**. Its first delay programs
`DD00000D`; the genuine ISR stops, ACKs3 and unlocks only when current exceeds
the guest deadline. A fixed222MHz SysClock / (221+1) is an explicit board input
choice **[М]**, not a captured electrical measurement.

`timers.cpp` implements only the captured clock/configuration profiles. Elapsed
Kermit peripheral ticks advance rational counters; reads do not advance time.
Comparison status bit1 latches, supplied W1C acknowledges it, and the existing
GIC delivers native IRQ135/141. Next-count equality/overdue expiry, explicit
counter/deadline or stop/start rearm, and output gating for the captured low
mode pattern are model choices **[М]**. Unknown active profiles stop with a
diagnostic; generic periodic/overflow/capture modes remain unsupported.

Seven focused tests include unchanged time providers, rollover, fractional
rates, byte lanes, physical GIC135 delivery and the complete native expiry/
lock helpers at equality and after one further tick. Integrated579 actually
reaches IRQ135 and SMC117 return0. Protocol and limitations:
`build/goal-native-systimer-proposal.md`,
`build/goal-native-timers-integrated-evidence.md`.
[Timer reference](https://www.psdevwiki.com/vita/Hardware_Timers) corroborates
the register layouts and clock selectors; it marks low mode bits as uncertain.

## 3. Ernie (RL78 syscon) **[Ф]/[Р]**

| Адрес | Что |
|---|---|
| `0x00000000..0x000FFFFF` | окно 1 МиБ в модели (`syscon::kFlashSize`); паспортные объёмы частей — `docs/SYSCON.md` §2. reset-вектор USS-1001 = `0xE000`, USS-1002 = `0x0DC00` |
| `0x000FFF00..0x000FFFFF` | SFR (и зеркало `0x0FFFFF00`) |
| `0xFFFA2` | handshake готовности железа: прошивка ждёт `0xC0` на `0x3005A` |
| `0xFFFA0..0xFFFA4`, `0xFFF04`, `0xFFF24`, `0xFFA3`, `0xFFF00..` | SP/PSW/PMC и порты, используемые reset-прологом |

## 4. eMMC

Реконструированная раскладка описана в `docs/EMMC.md`. Ключевые факты:

* контейнер **SLB2**: `"SLB2"`, version, `header_size = 0x200`, число записей,
  таблица записей с шагом `0x30` (`u32` смещение в блоках по 512 байт, `u32` размер
  в байтах, `u32` флаги, `u32` резерв, `char[32]` имя) **[Ф]**;
* записи 1.04: `second_loader.enp` (93184), `secure_kernel.enp` (33280),
  `kernel_boot_loader.self` (355220), `kprx_auth_sm.self` (35064),
  `prog_rvk.srvk` (1728) **[Ф]**;
* разделы `os0` и `vs0` — образы FAT16 (OEM `"SCEI"`), лежат в PUP готовыми **[Ф]**.

### 4.1 SDHCI (SDIF, 0xE0B00000): ADMA2 **[Р]**

Вторая стадия CMeP читает блоки **не через порт данных**, а через ADMA2:
`0x48094` строит таблицу, `0x48158` программирует её адрес и включает DMA.
Формат записи таблицы (8 байт, подтверждён кодом сборки таблицы и разбором
`0x45A3C`/`0x41AE4`):

| Смещение | Размер | Поле |
|---|---|---|
| `+0x00` | `u16` | attribute: бит 0 = VALID, бит 1 = END, биты 5:4 = ACT (`10b` = передача данных) |
| `+0x02` | `u16` | length, байт (не больше `0x10000` — так режет `0x480EC`) |
| `+0x04` | `u32` | физический адрес буфера |

Загрузчик пишет `attribute = 0x21`, последней записи — `0x23`. Таблица второй
стадии: база `0x40000400`, длина `0x100` (32 записи, до 2 МиБ на операцию),
первая запись — `{0x23, 0x200, 0x40000500}` для чтения блока 512 байт.

Регистры, которые при этом программируются: `0x04` block size (модель читает
биты 11:0 как размер, а старшую половину — как block count, `sdif.cpp`), `0x06`
block count, `0x08` argument (байтовое смещение: драйвер сам умножает номер
сектора на 512 в `0x470C4`), `0x0C` transfer mode (`0x0013` = DMA + block count +
read), `0x0E` command (`(index << 8) | flags`), `0x58` ADMA system address.

Поле `[desc+4]` дескриптора передачи: биты 0..2 = вид операции (4 = передача
данных, 1..3 = только команда), биты 3..7 = флаги команды (значение `0x10`
даёт `2` — короткий ответ, `0x28` → `27` — короткий ответ + CRC + busy),
бит 8 = чтение, бит 10 = DMA, бит 31 = «операция завершена».

### 4.2 Аргумент CMD17/18 у eMMC высокой ёмкости **[Р]**

Карта отвечает на CMD1 словом с установленным битом 30 (card capacity status),
поэтому аргумент CMD17/18/24/25 — **номер блока**, а не байтовое смещение.
Подтверждение: вторая стадия CMeP читает `ARG = 0x200` и проверяет маркер
`0xFFF5` в первых 32 полусловах блока (`0x46632`); в дампе eMMC
(`emmcdump.bin`) этот маркер лежит по байту **0x40000 = блок 0x200**, то есть
прочитан именно блок 0x200. Модель (`Sdif::current_lba`) следует спецификации;
байтовое чтение того же поля доступно через `ZLB_SDIF_BYTE_ARG=1`.

### 4.3 idstorage (user area 0x40000, 512 КиБ) **[Ф]**

Формат по дампу: лист 0 начинается 64 байтами `0xFFF5`, за ними таблица
полуслов `0x0000..0x007F`, затем `0xFFFF` до +0x180 и короткий список листьев
(`0x0080`, `0x0100`, `0x0102`, `0x0103`, `0x0110..0x0115`). Остальные листья
стёрты (`0xFF`). Загрузчик ищет в расшифрованной записи полуслово `0x0080`
(`0x4678C`) — то есть идентификатор листа из этого списка.
