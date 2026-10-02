# Цепочка загрузки прошивки 1.04

Документ описывает, что именно эмулируется, на основании чего это установлено и
где кончаются факты и начинаются модельные допущения. Текущее состояние и метрики
— `docs/STATUS.md`; подробности по звеньям — `docs/FIRST_LOADER.md` (CMeP
first_loader по шагам, с листингами и измерениями), `docs/KBL.md` (ARM-загрузчик
`kernel_boot_loader`), `docs/NSKBL.md` (небезопасный загрузчик ядра), `docs/SYSCON.md`
(Ernie/RL78).

## 0. Что есть в материалах

| Артефакт | Что это | Как используется |
|---|---|---|
| `dumps/vita_prototype_bootrom.bin` | first_loader CMeP (DEM, 16 КиБ, база `0x5C000`) | исполняется как есть |
| `dumps/pch-5c-cold_first_loader.bin` | retail-снимок first_loader | второй набор для проверки: заголовок он принимает, но останавливается позже — на RSA, потому что dev-ключ собран под ROM-константы прототипа (`docs/FIRST_LOADER.md` §17) |
| `dumps/bootrom_analysis/` | разбор first_loader | карта регистров и логика |
| `ernie-master/USS-1001.bin` | прошивка syscon (RL78, 1 МиБ, reset `0xE000`) | исполняется как есть |
| `Vita_104_Firmware/PSP2UPDAT104.PUP` | пакет обновления 1.04 | источник разделов |
| `Vita_104_Firmware/Out/SLB2/` | 7 записей: `second_loader.enp/.enc`, `secure_kernel.enp/.enc`, `kernel_boot_loader.self`, `kprx_auth_sm.self`, `prog_rvk.srvk` | содержимое SLB2 |
| `Vita_104_Firmware/Out/SLB2_dec/` | расшифрованные `second_loader.bin`, `secure_kernel.bin`, сегменты KBL | эталон для проверки крипто-цепочки |
| `Vita_104_Firmware/Out/PUP_dec/os0.bin`, `vs0.bin` | образы FAT16 разделов | содержимое пользовательской области eMMC |

## 1. Питание

1. Ernie (RL78) стартует с вектора `0xE000` из `USS-1001.bin`. Reset-пролог:
   `sel rb0 / movw sp,#0xFE20 / clrb !0x02F0 / mov !0x00F5,#0x80 /
   clrb !0x0078 / call !0x104F5`, затем инициализация `0x30035` и цикл ожидания
   `cmp !0xFFFA2,#0xC0 / bnz` на `0x3005A`.
   *Факт*: адреса и поведение восстановлены по дампу. *Модель*: значение `0xC0`
   в SFR `0xFFFA2` выставляет наш SFR-слой, то есть «железо готово»; счётчик SFR
   тикается из `Vita::run_slice()`, поэтому прошивка проходит цикл (pc `0x3005E` →
   `0x363F`, дальше свой рабочий цикл) и обслуживает SC-команды.
2. Ernie снимает сброс с CMeP (`ErnieBlock::release_soc`).

## 2. Роль «ARM boot ROM» (модель)

Дампа ARM boot ROM нет, поэтому этот шаг реализован на C++
(`Vita::arm_boot_rom_stage_second_loader`):

1. читает контейнер **SLB2** из загрузочного раздела eMMC (`boot0`, затем `boot1`,
   затем копии в пользовательской области `0x800000`/`0xC00000`);
2. берёт запись `second_loader.enc` (запасной вариант — `second_loader.enp`) и
   кладёт её в DRAM по физическому адресу `0x407C0000` — **не** в общую SRAM
   `0x1F000000`, потому что вторая стадия строит там `SceKblParam`;
3. пишет в mailbox `0xE0000010` значение `PA | 1` (бит 0 = «образ готов»);
4. сохраняет исходные SCE-байты auth-модулей из SLB2. Модель размещает их в общей
   DRAM только при переходе second_loader → secure_kernel, после использования
   этой области как IdStorage scratch: optional `context_auth_sm.self`, затем
   `kprx_auth_sm.self` и `prog_rvk.srvk`, с выравниванием 512 байт в пределах
   `0x40000500..0x40020000`. Для имеющейся 1.04 это `kprx_auth_sm` по
   `0x40000500` (размер `0x88F8`) и RVK по `0x40008F00` (`0x6C0`). Отсутствующая
   запись не занимает место; модельный `SceKblParam` содержит фактические PA/размеры.
   Это стенд-ин финальных чтений второй стадии, а не загрузка auth-файлов поверх
   приватной SRAM secure_kernel;
5. при `config_.provision_keys` подкладывает в CMeP синтетическую таблицу ключей
   `0xE0066000` и переподписывает staged-образ (`machine/bootkeys.cpp`,
   `tools/make_boot_keys.py`), чтобы RSA-проверка первого загрузчика выполнялась
   по-настоящему (сам ключ чипа в дампах отсутствует).

Соответствие трактовке из `ANALYSIS.md` §4.2: режим `'A'` = bit0 + bit6, при
установленном bit0 образ берётся от ARM.

## 3. CMeP first_loader (исполняется по-настоящему)

Точка входа `0x5C000`, окно CMeP RAM `0x40000..0x5FFFF` (128 КиБ),
`$0` (будущий `$sp`) = `0x60000`, аргумент `main` = `0x40000`.

```
_start (0x5C004)  очистка .bss [0x5EB00,0x5EE3C), $gp=0x5EB00, $tp=0x5E820
main    (0x5C4FE) init_emmc_keyrings -> check_boot_mode -> выбор пути
   check_boot_mode (0x5C0EE)
       *(0xE0030028) = 0x501            запрос флагов keyring 0x501
       $12 = *(0xE003002C); if (($12 & 3) != 3) return -1
       $12 = *(0xE0062020); return ($12 & 1) ? 'A' : '!'
   путь bit0 (0x5C574)
       ждать (*(0xE0000010) & 1)
       PA = *(0xE0000010) & ~3
       memcpy(0x40000,        PA,        64)
       validate_header(0x40000)
       memcpy(0x40000 + 64,   PA + 64,   size + field_0x10 - 64)
   process_image (0x5CC00)
       img_proc_* : keyring 8/9/0x0A/0x20, RSA через Bignum, AES через Bigmac,
                    массовая инвалидация флагов 0x1C1F/0xFC1F
   успех: keyring_set_value(0x501) и *(0xE0000000) = 1
   отказ: *(0xE0000000) = 2 и вечный цикл
```

В измеренном прогоне гость вычисляет boot mode = `0x21` (`'!'`): `StrapDevice`
отдаёт `0xE0062020` с битом 0 сброшенным. Поле модели `CmepBlock::boot_mode_`
(`0x41`) — другое значение и в отчётах печатается именно оно; оба режима содержат
`bit0`, поэтому образ приходит от ARM через mailbox (подробности — `docs/FIRST_LOADER.md`
§3 и §16).

### 3.1 Проверка заголовка (`validate_header`, `0x5C61C`)

| Смещение | Проверка |
|---|---|
| `+0x00` | магия `0x64B2C8E5` |
| `+0x04` | `size >= 0x2B0` и `size < 0x10000` |
| `+0x08` | `offset`; `+0x0C` = `length`; требуется `size == offset + length + 0x2B0` |
| `+0x10` | `field`; требуется `field + size < 0x1C000` |
| `+0x16` | `u16` < `0x10` |

Для 1.04 (`Out/SLB2/second_loader.enp`) эти поля равны: magic `64B2C8E5`,
`size = 0x2C0`, `offset = 0x10`, `length = 0`, `field_0x10 = 0x16600`,
`u16@0x16 = 0` — заголовок проходит все проверки, тело = `0x16600` байт.

Затем `_start` кладёт `$lp = 0x40000`, копирует 32-байтовый stub в `0x5FFE0`,
обнуляет слово по `0x5C000` (самоуничтожение точки входа) и делает `ret`, то есть
переход на `0x40000`.

### 3.2 Цепочка проверок проходит целиком

Прогон рапортует `first loader reported SUCCESS to the ARM mailbox` и передаёт
управление второй стадии. Что для этого потребовалось:

| Шаг | Адрес | Что делает | Как смоделировано |
|---|---|---|---|
| `validate_header` | `0x5C61C` | проверки контейнера ENP | реальный код |
| SHA-256 заголовка | `0x5C7F6` | `0x5CCF0` = SHA-256, `$1`=куда, `$2`=откуда, `$3`=длина, `$4`=32 | `BigmacFunction::Sha256 = 0x0033` |
| RSA-проверка | `0x5C83A` | 64 слова из `image+0x1C0` в окно Bignum, модуль из `0xE0066000` | ключ разработчика, подпись перезаписывается (`machine/bootkeys.cpp`) |
| digest-таблица | `0x5C9BA` | 32 байта `image+0x1A0` через keyring 10 → `0x5EDE0`, затем `memcmp` с SHA-256 тела | `0x030A` + провижининг записи `+0x1A0` |
| keyring-слоты | `0x5C904/0x5C93A` | загрузка слотов 10..15 из образа | `0x030A` (бит 28 = «+0x04 это индекс»), слоты однократно программируемые |
| распаковка образа | `0x5CBCC` | `0x010A`: AES-128-CBC из `image+0x2C0` **в** `image` (`0x40000`), ключ — слот 10 | **реальный AES** |
| хвост окна | `0x5CC86` | `0x000C`: операция по месту над `[image+0x16600, 0x1C000)` | модель |
| самоуничтожение | `0x5C000` | stub в `0x5FFE0` затирает `0x5C000..0x5FFE0` нулями и делает `ret` на `$lp = 0x40000` | реальный код |

**Ключ загрузки.** Слот keyring 10 — «fused» ключ чипа: `ENC_KEY || ENC_IV`
(`AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA`, `AF5F2CB04AC1751ABF51CEF1C8096210`). Именно им
`0x5CBCC` расшифровывает `image+0x2C0` в `0x40000`; проверено в Python: AES-128-CBC
от тела `second_loader.enc` этим ключом даёт ровно `Out/SLB2_dec/second_loader.bin`,
чей SHA-256 = поле `+0x20` заголовка ENP. Отсюда два следствия:

* ARM boot ROM должен ставить в DRAM **`second_loader.enc`** (не `.enp`);
* слот 10 нельзя перезаписать: загрузка из `image+0xE0` (`0x5C904`) игнорируется,
  как и положено однократно программируемому слоту.

`tools/make_boot_keys.py` генерирует dev-ключ под тот образ, который реально
ставится, и подписывает блок
`00 01 FF*203 00 || ROM[0x5E774:18] || SHA-256(header[0:0x1C0])`, куда входит и
запись digest-таблицы, поэтому все `memcmp` загрузчика выполняются по-настоящему.

## 4. CMeP second_loader

Исполняется из `0x40000` (образ уже расшифрован туда первым загрузчиком). Читает
контейнер SLB2 с eMMC (в измеренном прогоне ~4121 чтение), готовит ARM-контекст
(в том числе `SceKblParam` по PA `0x1F000100`) и заканчивается вызовом сервиса
первого загрузчика:

```
0x403C0..0x40404  обнуление регистров, $1..$5 = адреса Bigmac ($0xE005001C/24/9C/A4)
                  и ящика ($0xE0000000), затем jmp 0x5FF00
```

Обмен с syscon идёт по SPI0 к Ernie (`0xE0A00000`) и через окно SC
(`0xE0B00000` запрос / `0xE0BF0000` ответ); регистры `0x30000118`, `0x30000208`,
`0xE0064060` читает **первый** загрузчик (`boot_path_bit5`, `0x5C206`), а не вторая
стадия.

`0x5FF00` лежит в окне первого загрузчика, но **ни в одном** из двух дампов
исполняемого кода там нет: в прототипном (`vita_prototype_bootrom.bin`) весь
участок `0x5FEF0..0x5FFFF` — нули, в retail-снимке
(`pch-5c-cold_first_loader.bin`) — таблица абсолютных указателей
(`0x5EDA0, 0x5FF6C, 0x5D7DA, 0x5D276, …`), которые наводят на уже затёртые
адреса (`0x5EDA0` лежит в `.bss` этого же снимка). Разбор вызова уточняет, что
`0x5FF00` — не вход сервиса, а **буфер запроса**: вторая стадия сама пишет туда
32 слова (`0x403A2..0x403B4`, `repeat $2 = 31` → `0x5FF00..0x5FF7F`, источник —
`0x40100` её собственного образа), обнуляет регистры и уходит туда `jmp`
(`0x40404`). Сама роутина, которая этот запрос читает и выбирает сервис, — код
boot ROM (на-chip, дампа нет) либо retail-сборки первого загрузчика, которую наша
цепочка не исполняет. Поэтому вызов перехватывается `MePCore::pc_hook` и
заменяется шагом 4 вики — перезапуском CMeP на `secure_kernel.enp`
(`Vita::serve_cmep_service_call`, §4 в `docs/STATUS.md`).

Проверяется это прогоном **без** подстановок: роутина не найдена, исполнение
входит в скопированный дескриптор и упирается в `sleep` на `0x5FF1E` (это байты
самой второй стадии из образа, `62 70`). Регистры на входе —
`$1=0xE005001C $2=0xE0050024 $3=0xE005009C $4=0xE00500A4 $5=0xE0000000
$6=0x500`, `$lp=0x4039B`:

```powershell
$env:ZLB_NO_SUBSTITUTION=1
build\bin\zeliboba.exe -q -ex "boot" -ex "runm 300000" -ex "boot" -ex "gpo" -ex "quit"
# → CMeP halted: sleep instruction, pc=5FF1E, 262 777 инструкций MeP
# → GPO raw=0x00540000 checkpoint=0x54
```

Порядок GPO в этом прогоне [Р]: `0x41 0x42 0x43 0x44 0x45 0x48 → 0x54` (с
подстановками тот же ряд и дальше `0x46 0x47 0x49 0x4D 0x51 0x54 0x55 0x56
0x57 0x58 0x5F`). Покрытие второй стадии: **4883 / 45824** сайта (10.7 %) без
подстановок против **9559** (20.9 %) с ними.

Отдельно измерено (`--first-loader dumps\pch-5c-cold_first_loader.bin`, режим
«без подстановок»): retail-сборка **исполняется в модели целиком** — проходит свои
проверки, сообщает `SUCCESS` в ящик и передаёт управление второй стадии (на
`0x402FA`, 263 856 инструкций MeP). То есть missing-ROM-заглушка совместима с
обеими сборками, а полного прогона не хватает только сервисной точке `0x5FF00`,
которой нет ни в одном снимке. Раскладку констант retail-сборки модель определяет
сама — `detect_first_loader_layout` в `src/machine/bootkeys.h` (сдвиг `.data`
−0x80; маркер — 16 байт `A7 E6 05 63 …` перед DER-блобом).

На пути к этому концу вторая стадия упиралась в SC-регистры; в модели их пришлось
довести: регистр событий `0xE31000C0` (CMeP ждёт ровно `1`), «защёлка» статуса
`0xE3101000` (CMeP пишет `0x0001000F`, позже `0`), окно `CMeP.SecureCtl`
`0xE3100000..0xE311FFFF` (128 КиБ) — вторая стадия трогает `0xE3110C00`. Регистр
`0xE3102100` (строб SC в `0x49536..0x49544`) модель **удерживает** на записанном
значении `1`: принудительный `0` зацикливал CMeP в `erepeat` навсегда
(`src/hw/cmep/mailbox.cpp`).

## 5. CMeP secure_kernel

`secure_kernel.enp` слинкован на `0x800000` (его первые слова — векторная таблица
абсолютных прыжков `jmp 0x800100`, `jmp 0x80028C`, …), это приватное окно CMeP
`CMeP.Private` (2 МиБ). Ниже порядок проверенного полного прогона с 579 проходящими
тестами (`build/goal-native-emc-syscon-full.log`):

1. стартовые проверки keyring/sysctl/версии;
2. стенд-ин ARM boot ROM подтверждает оставшийся статус второй стадии `0x9`;
3. CMeP публикует `0x101`, модель отпускает ARM для запуска настоящего
   SceSblSmsched. Статус остаётся pending: гость подтверждает его записью `0x101`,
   затем пишет свой shared-buffer PA `0x401402C0` и отдельно doorbell `1`;
4. CMeP получает `0x401402C1`, сохраняет PA и публикует `0x102`, который
   подтверждает настоящий ARM scheduler. MeP засыпает на `0x800488` (PC `0x80048A`),
   state `3`. ARM посылает команду `0x80A01` для RVK.
5. Source 8 пробуждает MeP; настоящий вектор `0x800050` и обработчик
   `0x800A42` принимают и подтверждают команду. Bigmac исполняет DMA,
   AES-256-CBC `0x238A`, AES-128-CBC `0x218A` и native SHA-256 `0x2093`.
   Plain-section copy `0x2080`, HMAC-SHA256 `0x20B3` и AES-128-CTR `0x21A1`
   также реализованы. Настоящая RSA-проверка и оба section HMAC проходят;
   RVK callback возвращает `0`, гость публикует 21 запись по `0x809410`.
   CTR plaintext совпадает с независимой распаковкой, counter обновляется
   на 42 блока. Прежний отказ `0x800F0627` устранён.
6. Независимый `0x80901` регистрирует `{PA 0x40140200, length 0x80}` и
   возвращает настоящий статус `1`. SceSblSmsched запускается со start `0`,
   выставляет `0x5190A0 = 1` и регистрирует SMC `0x12D..0x13C`.
7. GIC доставляет mailbox Group0 как Secure FIQ на ARM3. Исправлен reset SPI
   по Cortex-A9 (`ICDICFR = 0x55555555`, level-sensitive), поэтому polling ACK
   не оставляет stale pending и прежний Smsched panic исчез. Настоящий SGI4
   доставляет completion в NSKBL и подтверждается EOI token `0xC04`.
   Public retail prefix в slot `0x509` (`0xE0062120`) устраняет прежний отказ
   platform/RVK policy `0x800F0B31`; это моделируемый input, не console dump.
   Payload CTR и streamed HMAC `0x24B3/0x2CB3/0x28B3` проверены:
   payload return `0`, outer launch `1`, настоящий entry `0x80B000` исполнен.
   Plaintext и final digest совпадают с независимой ELF/metadata проверкой.
8. Native kprx startup использует SWI0 для dispatcher registration. Доставка
   SWI теперь исправлена: vector `0x800014`, return `1`, IRQ9 callback `0x80E55E`.
   Модуль остаётся в настоящем work loop `0x80E84A`. DRAM-size slot `0x513`
   теперь согласован с board 512 MiB. First os0 services `0x10001/0x20001/0x30001`
   возвращают `0`, section plaintext — валидный zlib stream `0x2DD` bytes.
   Исправленный unaligned Thumb32 addressing позволяет настоящему inflater
   вернуть `0x658`, затем `0x318`; оба outputs точно совпадают с ELF load/relocation
   segments. Все segment loads и native relocation возвращают `0`.
   Stop/exit notification `0x10000` завершено. Затем cleanup вызывает low-SRAM
   helper `0x400CE`: его настоящий код есть в префиксе secure_kernel по `0x8000CE`.
   Низкое окно теперь alias-ит private SRAM на board handoff, helper возвращает
   `0` на `0x80020E`. Native Bigmac zero-fill `0x000C` очищает module arena,
   не меняя kernel. Native restart публикует `0x102` и засыпает на `0x80048A`.
   Затем гость загружает все 28 bootconfig list modules и запускает все 14
   core modules плюс Stdio и Lowio. Native threads работают; следующая граница
   — Secure hardware polling (§7);
   native display contract — `FIRMWARE_DISPLAY_104.md`.

За этот полный прогон CMeP исполняет ~38 млн инструкций и завершает cleanup.
Mailbox-прерывания, SWI и реальный RETI работают;
подробности ControlBus, masks/priorities и vector remap — `HARDWARE.md` §1.2.2.
Оба mailbox endpoint реализуют set исходящих битов и clear подтверждённых
входящих битов; последовательность PA, затем `1` сохраняет адрес.

*Модель:* отсутствующий reset/ROM/SC-путь отпускает ARM через `release_soc()`
после `0x101`. Контекст перед KBL по-прежнему дополняется моделью.
Старое принудительное `state = 9` удалено: это запрос завершения и очистки памяти
secure kernel. Его эпилог ошибочно читал `0x102` из переключённого стека как адрес
возврата; выполнение unmapped памяти до `0x40002` было ошибочно названо «done».
Прогон больше не использует этот признак успеха или автоматические ответы `1`.

## 6. ARM kernel_boot_loader

`kernel_boot_loader.self` — SELF с `self_type = 0x09 (BOOT)`, `keyrev = 1`;
`self_to_elf` расшифровывает 4 секции AES-128-CTR и воспроизводит
`Out/SLB2_dec/kernel_boot_loader.self.seg00..seg03` побайтово. Точка входа
`0x40020000`; код работает на **всех четырёх** ядрах Cortex-A9 и синхронизирует их
барьером `0x4003B384` (счётчик `0x4005C008`, `strexh`).

Подтверждённый ход (адреса и детали — `docs/KBL.md`):

* печатает в UART баннер `Starting PSP2 Kernel Boot Loader [0x 01040011]: 300`;
* сам распаковывает NSKBL своими `sceArlzDecode` (`0x4003C330`) и
  `sceArlzArmFilter` (`0x4003CB40`), поток `0x50000004` → `0x51000000`;
* рапортует чекпойнты `0x88`, `0x89` и уходит в Non-Secure через `0x40021AE0`
  (`SCR.NS = 1`), после чего NSKBL стартует в небезопасном мире;
* **читает** `SceKblParam` (PA `0x1F000100`): `0x400376E4` копирует 128 байт в
  `0x400B2DC8` → `0x40300100`, плюс поле `+0xC4` читает развилка `0x4002028C`.
  В текущем обычном прогоне C++-модель строит ненулевую запись после зеркалирования
  scratchpad: magic `0xCBAC03AA`, DRAM `0x40000000+0x20000000`, реальные адреса
  и размеры auth-файлов (§2), sleep `0x60`, wakeup `0xFF14`. Это модель параметров
  предшествующей стадии, не доказательство исполнения гостевого сборщика `0x41B4A`.

Исторический замер с `ZLB_RTRAP=0x100-0x1D0` обнаруживал нулевую запись:
second_loader исполнял очистку `0x41B38`, но не `0x41B4A`, а C++-сборщик тогда
не вызывался при включённых подстановках. Вывод той серии о проходе KBL с пустым
параметром относится к старому состоянию модели; текущий порядок построения
и зеркалирования исправлен (`docs/STATUS.md` §4).

*Модель:* карта памяти, которую KBL наследует от предыдущей ступени (низкое окно
ARM, начальная страница векторов `PA 0x16100`), и отдельные fallback-хелперы —
подстановки (`docs/STATUS.md` §4). Созданные гостем Monitor-векторы, валидные
dlmalloc heaps и ненулевая native-таблица MemBlock сохраняются. Старый page-cache
stand-in и heap-lookup shortcut теперь opt-in, по умолчанию выключены.

## 7. NSKBL

NSKBL стартует на VA `0x51000000` и печатает свой баннер
(`Starting PSP2 Kernel Boot Loader (Non-secure)`). Подтверждённые чекпойнты
(писатель GPO `0x51010F98`): `0xA1` `0x510008DC` — pre-init, `0xA2` `0x5100094C`,
`0xA3` `0x51000C18`, `0xA4` `0x51000C22`, `0xA5` `0x510003CA`, `0xA6` `0x510004CA`;
достигнут **`0xA9`** `0x51000CF6` («kernel pre-init done, before first external
load»). GPIO остаётся `0xA9`; дальнейшие native module starts описаны ниже.

Что уже работает: NSKBL поднимает eMMC повторно, читает том `os0` (`lba=0`,
`lba=65536`, `lba=65568`, затем данные с `lba=72800`), разбирает FAT16 и читает
`os0:psp2bootconfig.skprx` через настоящий объект драйвера `0x5117CB00` и SDIF ADMA.
Полный прогон даёт 10753 sector reads, 0 writes; клон eMMC остаётся чистым.
Auth-команды `0x10001/0x20001/0x30001` завершаются transport `1`, service `0`.
Расшифрованный поток по VA/PA `0x5113C000` содержит `0x2DD` байт валидного zlib:
независимая распаковка даёт `0x658` байт и побайтово совпадает с первым PT_LOAD
`psp2bootconfig.elf`. Native ARM decoder теперь выдаёт точные load и relocation
segments; relocation, linking 12 imports и первый module callback проходят.
Прежние list-load errors `0x803FF007` исправлены: A32 word load больше не
портит `/kd/` при unaligned path copy. Оба списка возвращают `0`, все 28 modules
получают положительные UID и проходят native relocation. NEON initializer
ThreadMgr также исполняется после исправления D/Q register decode.
Все 14 core-module starts, Stdio, Lowio, Syscon, OLED, Display и SblSsSmComm успешны.
Exact reset7/idle I2C model
устраняет прежний Lowio wait на `0xE050001C`. Architectural TPIDR registers
сохраняют native per-core context pointers и security banks; threads работают.
Syscon VMOV/VST1 padding block также исправлен. LT5/WT7 считают elapsed time;
настоящий IRQ135 handler освобождает delay wait. Все шесть SceEmcTop commands
завершены, SMC117 возвращает `0`; CDRAM имеет независимый RAM backing.
GPIO248/sub4 теперь выполняет receive/ACK/stop. RFE Thumb return исправлен;
OLED A1 сохраняет SP и выдаёт точные18 bytes, отвергает моделируемый high input
и устанавливает ready2. Display WaitReady/script/head возвращают0, DSI0 включён.
Syscon checksum SIMD исправлен. Boot flags раннего modeled handoff теперь
согласованы с NVS: FF FF 00 FF, product4; поздний external marker удалён.
Native allocated per-core stacks сохранены, fallback только для shared top4000.
Все четыре Display predicates0; producer005B7048/gzip1FE000/SetFrameBuf0 и
vblank wait0 исполняются. Гостевой logo buffer PA1C000000 побайтово совпадает
с embedded asset. Deferred IFTU bank1 публикуется на следующем DSI frame;
physical IRQ204 вызывает настоящий Lowio handler, zero ACK, +180 rearm и replay
в старый bank0. Прогон: `runm 1000000`, `build/goal-native-iftu-arm-full.log`,
suite **617 / 0**; breakpoints: `build/goal-arm-native-syscon-oled-iftu-arm.log/.json`.
Белый PlayStation logo на чёрном фоне подтверждён в
native SDL screenshot (image omitted from source delivery).
Timing/rearm/status — ограниченная модель; подробности и пределы проверки в
[integrated evidence](../build/goal-native-iftu-arm-integrated-evidence.md).

Историческое наблюдение пустой структуры по VA `0x240`, отсутствия чтений файла
и результата `0x80320011` относится к ранним неполным путям инициализации
(`docs/NSKBL.md` §7); текущий native driver/completion путь это препятствие прошёл.

## 8. Ядро

ARM теперь исполняет настоящий код os0 kernel modules в DRAM: все 14 core
starts возвращают `0`, затем Stdio, Lowio, Syscon, OLED, Display и SblSsSmComm;
native threads работают.
Обычный cold path создаёт настоящий logo buffer, публикует его через IFTU и
показывает PlayStation logo в native SDL (§7). Полная загрузка ядра и LiveArea
ещё не наблюдались. Прежний interrupt context divergence исправлен.
Стадия `--stage kernel` грузит модуль ядра напрямую (`bootimage.elf` →
`0x81000000`) — это отладочный путь, позволяющий работать с ядром, не доводя
предыдущие ступени.

## 9. Разделяемая память (как в модели)

| Физический адрес | Кто видит | Назначение |
|---|---|---|
| `0x00000000..0x0003FFFF` | ARM | низкое окно, алиас power scratchpad (256 КиБ) |
| `0x00040000..0x0005FFFF` | ARM | зеркало CMeP SRAM `0x00800000` («MeP boot») |
| `0x00040000..0x0005FFFF` | CMeP | Boot backing first_loader и staging second_loader (128 КиБ); после secure-kernel handoff alias private SRAM `0x800000`, model boundary с неизвестным selecting register |
| `0x00060000` | CMeP | вершина стека (`$0` при сбросе) |
| `0x00800000..0x009FFFFF` | CMeP | приватная RAM CMeP: secure kernel и Secure Modules (2 МиБ) |
| `0x40000500`, `0x40008F00` | ARM и CMeP | исходные `kprx_auth_sm.self` и `prog_rvk.srvk` для имеющейся 1.04; общий staging-пул `0x40000500..0x40020000`, §2 |
| `0x1F000000..0x1F03FFFF` | ARM и CMeP | общая загрузочная SRAM (256 КиБ); `SceKblParam` — по `0x1F000100` |
| `0x1C000000..0x1C1FFFFF` | ARM | Scratchpad SRAM (2 МиБ) |
| `0x1D000000` | ARM | аппаратный «/dev/null» (4 КиБ) |
| `0x40000000..0x5FFFFFFF` | ARM и CMeP | DRAM, видимая обеим сторонам (KBL линкуется на `0x40020000`) |
| `0x80000000..0x83FFFFFF` | ARM | окно основной DRAM (64 МиБ) |
| `0x1A000000..0x1A001FFF` | ARM | native MPCore: SCU, GIC (CPU interface `+0x100`, distributor `+0x1000`), GT, PT |
| `0x1A002000` | ARM | native PL310 L2 controller |

## 10. Что подтверждено, а что модель

**Подтверждено материалом:** формат SLB2 (заголовок и таблица записей — сверено с
дампом CP), поля и проверки заголовка `.enp`, порядок и адреса MMIO first_loader,
reset-пролог Ernie, содержимое SLB2 1.04, формат FAT16 разделов `os0`/`vs0`, код
операции Bigmac (`0x0033` SHA-256, `0x030A` keyring, `0x010A` распаковка образа,
`0x000C` операция по месту, `0x0301` запись keyring), ключ загрузки
`ENC_KEY||ENC_IV` (даёт ровно `SLB2_dec/second_loader.bin`), самоуничтожение ROM
`0x5C000..0x5FFE0` перед передачей управления, распаковка NSKBL кодом самого KBL
(сверено с `unarzl` побайтово), чекпойнты KBL и NSKBL, содержимое тома `os0`.

**Модель (документированное допущение):** роль ARM boot ROM на C++; адрес
`0x40000000`/`0x407C0000` для стейджинга; точная раскладка MBR пользовательской
области; адресация периферии Kermit там, где её не удалось подтвердить реверсом
модулей прошивки; начальные значения батареи/RTC syscon; ключ разработчика в
таблице `0xE0066000` и переподпись образа (реальная таблица в дампы не попала);
запись digest-таблицы `+0x1A0`; handoff-стейджинг auth-файлов и построение
`SceKblParam`; карта памяти, которую KBL наследует от предыдущей ступени;
stage-boundary remap EVM=0 vectors (`0x40000` → `0x800000`, реальный управляющий
регистр ещё неизвестен); board-профиль public identity slot `0x509` и DRAM-size
slot `0x513`. MeP IRQ/SWI и native ARM mailbox completion теперь исполняют код
гостя; старые fake `state=9`, auto-reply `1` и «done» по `0x40002` удалены.
Текущие нерешённые пути — native ARM decompression error и low-SRAM helper alias
при cleanup, а не отсутствие структуры хранения NSKBL. Полный список подстановок —
`docs/STATUS.md` §4.
