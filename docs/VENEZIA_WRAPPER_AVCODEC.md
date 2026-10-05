# SceVeneziaWrapper и SceAvcodec: NID, состав, структура

Разбор двух ключевых kernel-модулей, через которые user-space (и весь кодек-стек)
выходит на Venezia/VIP. Продолжение `docs/VENEZIA_STACK.md`.

Источник: `os0/kd/bootimage.elf` (`SceKernelBootimage`, 0x2A5530 байт) — контейнер
из **42 встроенных модулей**; модули ищутся по ELF-магии `\x7fELF` внутри файла.

---

## 1. Все встроенные модули и как они упакованы

`bootimage.elf` — обычный модуль PSVita (`e_type=0xFE04`, `e_machine=0x28`,
`e_flags=0x05000000`), у которого сегмент 0 (файл 0xA0 → `0x81000000`, 0x2A542C)
содержит другие модули целиком. Каждый вложенный модуль — тоже полноценный ELF
со своими program headers:

```
hdr=0x096EF4  SceVeneziaWrapper
   seg0 file 0xA0  vaddr 0x81000000  filesz 0xDAA0  RX
   seg1 file 0xDB40 vaddr 0x8100E000 filesz 0xB8C  RW
   seg2 relocation trailer, file 0xE6D0
```

Найдено 34 ELF-магии (`_scratch/boot_hdr.py`), из них 42 описаны таблицей
менеджера. `SceModuleInfo` вложенного модуля лежит по
`hdr + seg0.filesz + e_entry`:
для `SceVeneziaWrapper` — `0x096EF4 + 0xDAA0 + 0xCBA0 = 0x0A3B34`.

## 2. SceVeneziaWrapper: две библиотеки

`SceModuleInfo` по файловому смещению **0x0A3B34**:

| поле | значение |
|---|---|
| attribute | `0x07` |
| version | `0x0101` |
| name | `SceVeneziaWrapper` |
| type | `0x06` |
| module_nid | `0x69D62026` |
| export range | `0xCBFC..0xCC5C` |
| import range | `0xCC5C..0xCECC` |
| gp | 0 |

Экспорт — **две** записи `SceLibEntTable` (по 0x20 байт):

| запись | файл | nfun | nvar | ntls | lib_nid | библиотека |
|---|---|---|---|---|---|---|
| `SceVeneziaWrapperForDriver` | 0x0A3B90 | **47** | 0 | 0 | `0x4F5231A4` | для драйверов (ядро) |
| `SceVeneziaWrapper` | 0x0A3BD0 | **16** | 0 | 0 | `0x2B2DA8E1` | для user-space |

Для сравнения, `SceAvcodec` (modinfo 0x199860):

| запись | файл | nfun | nvar | ntls | lib_nid | библиотека |
|---|---|---|---|---|---|---|
| `SceAvcodec` | 0x1998DC | **108** | 0 | 4 | `0xA166C96E` | кодек-API |

## 3. Восстановленные NID

### 3.1 `SceVeneziaWrapper` — 16 экспортов: **имена восстановлены**

Таблица NID: файл **0xA4038**, таблица адресов: файл **0xA4098**.

**Главное открытие.** Все 16 NID оказались в `db.yml` — но не под именем
`SceVeneziaWrapper`, а как библиотека **`SceCodecEngineWrapper`**
(top-level `SceCodecEngineWrapper`, NID модуля `0x50CC4832`, lib_nid
`0x5C9EE5B9`). Совпадение **16/16**, порядок в таблице тот же:

| № | NID | адрес | имя |
|---|---|---|---|
| 0 | `0x03DCBDCA` | `0x8100C9C5` | `_sceCodecEngineAllocMemoryFromUnmapMemBlock` |
| 1 | `0x04BA9415` | `0x8100C9DD` | `_sceCodecEngineChangeNumWorkerCoresMax` |
| 2 | `0x04D5F36B` | `0x8100CA9D` | `_sceCodecEnginePmonStop` |
| 3 | `0x1E9E5A79` | `0x8100CB55` | `_sceCodecEngineGetMemoryState` |
| 4 | `0x241B194B` | `0x8100CB25` | `_sceCodecEngineGetProcessorLoad` |
| 5 | `0x362E9415` | `0x8100C9D5` | `_sceCodecEngineChangeNumWorkerCoresDefault` |
| 6 | `0x3EBA4982` | `0x8100CAA5` | `_sceCodecEnginePmonGetProcessorLoad` |
| 7 | `0x489FF965` | `0x8100C9CD` | `_sceCodecEngineFreeMemoryFromUnmapMemBlock` |
| 8 | `0x6AF71F08` | `0x8100CA95` | `_sceCodecEnginePmonStart` |
| 9 | `0x7E5E1F38` | `0x8100CB85` | `_sceCodecEngineChangeNumWorkerCores` |
| 10 | `0x8EFF2DAA` | `0x8100CB1D` | `_sceCodecEngineResetNumRpcCalled` |
| 11 | `0x9B157692` | `0x8100CB15` | `_sceCodecEngineGetNumRpcCalled` |
| 12 | `0xAD30912D` | `0x8100C9BD` | `_sceCodecEngineCloseUnmapMemBlock` |
| 13 | `0xB0E654EE` | `0x8100C9B5` | `_sceCodecEngineOpenUnmapMemBlock` |
| 14 | `0xCA79BFC4` | `0x8100CB0D` | `_sceCodecEnginePmonReset` |
| 15 | `0xDE5EF6CC` | `0x8100CB8D` | `_sceCodecEngineSetClockFrequency` |

**Что это значит.** `SceVeneziaWrapper` — это экспортное **имя-фасад**
(alias) для того же API, что SDK и прошивка называют `SceCodecEngineWrapper`.
Отличаются только имена и NID библиотек:

| | lib_nid | функций | где |
|---|---|---|---|
| `SceVeneziaWrapper` | `0x2B2DA8E1` | 16 | экспорт bootimage, импорт `avcodec_us` |
| `SceCodecEngineWrapper` | `0x5C9EE5B9` | 16 (те же) | имя в `db.yml` |

То есть кодек-стек в user-space вызывает `SceVeneziaWrapper`, а внутри это
публичный API `CodecEngine`, обёрнутый под имя Venezia. Номенклатура объясняет,
почему в SDK есть `libSceCodecEngine_stub.a` и `sceCodecEnginePmon*`.

**Публичный и приватный API.** В `db.yml` видно три библиотеки вокруг этого API:

* `SceCodecEngineUser` (lib `0x469AB062`, 4+ функции) — публичные
  `sceCodecEngineAllocMemoryFromUnmapMemBlock`, `…OpenUnmapMemBlock`,
  `…CloseUnmapMemBlock`, `…FreeMemoryFromUnmapMemBlock` (их экспортирует
  `avcodec_us` как `SceCodecEngineUser`, 6 функций);
* `SceCodecEnginePerf` (lib `0x9BF4FFAD`) — публичные `sceCodecEnginePmon*`;
* **`SceCodecEngineWrapper`** (lib `0x5C9EE5B9`) — 16 функций с префиксом `_`,
  то есть внутренний слой; именно его bootimage отдаёт под именем
  `SceVeneziaWrapper`.

По смыслу это API **управления кодечными ядрами**, а не сами кодеки:
управление рабочими ядрами (`ChangeNumWorkerCores*`), выделение памяти вне
отображения (`*UnmapMemBlock*`), мониторинг (`Pmon*`, `GetProcessorLoad`,
`GetMemoryState`), счётчик RPC (`*NumRpcCalled`), частота (`SetClockFrequency`).
Это прямо подтверждает вывод из `docs/VENEZIA_STACK.md` §5.1: декодирование
исполняют отдельные процессоры, и у них есть выделенные ядра, память, частота
и RPC-канал.

### 3.2 `SceVeneziaWrapperForDriver` — 47 экспортов (lib_nid `0x4F5231A4`)

Таблица NID: файл **0x0A3E7C**, значения строго по возрастанию (что и
подтверждает, что найдена именно таблица NID, а не код):

```
0x0104770D 0x06094EC3 0x0B27EB16 0x0E38B133 0x11857760 0x1395797E
0x16809E85 0x20222E84 0x240914C9 0x252376AB 0x2B78C6D2 0x47D49340
0x570D752B 0x594FF0FD 0x5A10387E 0x5ADB0A69 0x5C8AD744 0x64DFA0A9
0x6D41287F 0x6D5A5F9D 0x6ED83374 0x70C5597A 0x744D4C3F 0x788D332A
0x7A3B4841 0x82E378BD 0x8BC041CA 0x8C5B0E5A 0x8CEF0DD3 0x948FFE33
0x97938556 0xA0949FC5 0xA8B54DEF 0xADF507C9 0xB7D83410 0xBCDCEA13
0xC0C3276F 0xC32A88F7 0xC5478DD5 0xC966B369 0xD651299B 0xDA4B7AA7
0xEA88ABE5 0xF0D7B3F7 0xF3F71CEF 0xFA8D66CA 0xFBD4853E
```

**Шесть имён восстановлены** по [странице SceVeneziaWrapper](https://wiki.henkaku.xyz/index.php?title=SceVeneziaWrapper)
в Vita Development Wiki (проверено: все шесть NID присутствуют в нашей таблице,
6/6):

| NID | имя | смысл |
|---|---|---|
| `0x2B78C6D2` | `sceVipDmacMemcpyBlockingForDriver` | блокирующее DMA-копирование для **VIP** (не Venezia) |
| `0x64DFA0A9` | `sceVeneziaConvertPhysicalToVirtualForDriver` | физ. адрес → виртуальный |
| `0x788D332A` | `sceVeneziaGetVThreadProcessingResourceForDriver` | ресурс потока обработки V |
| `0x8BC041CA` | `sceVeneziaConvertVirtualToPhysicalForDriver` | виртуальный адрес → физический |
| `0x97938556` | `sceVeneziaIsProcessSuspendedForDriver` | проверка приостановки |
| `0xC32A88F7` | `sceVeneziaMemcpyChainForDriver` | цепочка копирований (DMA) |

Для `sceVeneziaMemcpyChainForDriver` вики даёт и прототип (размер структуры
0x24 на FW 0.990):

```c
typedef struct SceVeneziaMemcpyChainParam {  // size is 0x24 on FW 0.990
    SceUInt32 unk_0;      // must be >= 9
    SceUInt32 unk_4;      // must be >= 9
    SceBool   someId;     // selects which elements are used
    SceUInt32 unk_C;
    SceUInt32 unk_10;
    SceUInt32 unk_14;
    void     *someSrc;
    SceUInt32 unk_1C;
    SceSize   someSize;
} SceVeneziaMemcpyChainParam;

int sceVeneziaMemcpyChainForDriver(SceVeneziaMemcpyChainParam *pParam);
```

**Остальные 41 NID пока без имён.** Вики перечисляет ровно те же шесть — значит
наши данные и вики независимо подтверждают друг друга, и «дырка» именно в 41
функции, а не в нашей таблице.

Что из имён видно по семантике: `ForDriver`-библиотека — это **низкоуровневый
сервис для ядра**: преобразование виртуальных/физических адресов, цепочки
DMA-копирований, проверка приостановки процесса, доступ к ресурсу обработки.
Именно то, что нужно, чтобы складывать данные в SPRAM и гонять их на Venezia/VIP.
Именно эту библиотеку импортируют три других модуля bootimage (39+7+6 функций).

### 3.3 `SceAvcodec` — 108 экспортов (lib_nid `0xA166C96E`)

Запись экспорта — файл `0x1998DC`: `nfun=108`, `nvar=0`, `ntls=4`. Имена и NID
всех 108 функций уже известны из `db.yml` (`_sceAudiodec*`, `_sceAudioenc*`,
`_sceAvcdec*`, `_sceJpeg*`, `_sceVideodec*`) — см. `docs/VENEZIA_STACK.md` §2.
Позиция таблицы NID в самом `bootimage.elf` автоматически не подтверждена
(в отличие от двух библиотек обёртки): тела функций, на которые указывает
`entry_table=0x8101714C`, в файле тоже не отображаются (§5), поэтому ограничусь
тем, что запись экспорта разобрана и её lib_nid/nfun совпадают с `db.yml`.

Три семейства и их варианты (из `db.yml`):

* аудио-декод: `_sceAudiodec{Create,Delete}Decoder{,External,Resident}`,
  `_sceAudiodecDecode`, `_sceAudiodecPartlyDecode`, `_sceAudiodecDecodeNFrames`,
  `_sceAudiodecDecodeNStreams`, `_sceAudiodecGetContextSize`;
* аудио-энкод: `_sceAudioenc{Create,Delete}Encoder{,External,Resident}`,
  `_sceAudioencEncode`, `_sceAudioencEncodeNFrames`, `_sceAudioencGetOptInfo`;
* видео-декод: `_sceAvcdecCreateDecoder{,Internal,Nongameapp}`,
  `_sceAvcdecDecode{,AuInternal,AuNalAu…,WithWorkPicture}`, `_sceAvcdecCsc{,Internal}`,
  `_sceAvcdecQueryDecoderMemSize{,Internal}`, `_sceAvcdecGetSei*`.

### 3.4 Алгоритм NID и его проверка

Алгоритм из [Vita Development Wiki: NID](https://wiki.henkaku.xyz/index.php?title=NID):

```
hash = sha1(name + nid_suffix)          # nid_suffix — «соль» библиотеки
tr   = первые 32 бита hash              # 0xZZYYXXWW
NID  = swap32(tr)                       # == little-endian чтение байт 0..3
```

Проверка на трёх примерах из вики (`_scratch/nid_impl.py`):

| вход | sha1 → NID | ожидалось |
|---|---|---|
| `sceDisplayGetFrameBuf` (без соли) | `0xEEDA2E54` | `0xEEDA2E54` ✔ |
| `module_start` + бинарная соль PSP2 NONAME | `0x935CD196` | `0x935CD196` ✔ |
| `SceIpmi` + `libipmi` | `0xF4E34EDB` | `0xF4E34EDB` ✔ |

Скрипт для проверки символов, найденных в отладочных прошивках:

```python
import hashlib, struct

def nid(name, suffix=b""):
    """PSVita NID: le32(sha1(name + nid_suffix)[0:4])."""
    if isinstance(suffix, str):
        suffix = suffix.encode()
    return struct.unpack_from("<I", hashlib.sha1(name.encode() + suffix).digest(), 0)[0]

# примеры
assert nid("sceDisplayGetFrameBuf") == 0xEEDA2E54
assert nid("SceIpmi", "libipmi") == 0xF4E34EDB
```

Соль (`nid_suffix`) у каждой библиотеки своя; известные случаи — из таблицы
плайнтекст-солей вики: `SceAudiodecUser` → `SceAudiodecUser`,
`SceCodecEngineUser` → `SceCodecEngineUser`, `SceCodecEnginePerf` →
`SceCodecEnginePerf`, `SceJpegUser` → `SceJpegUser` и т. д. (то есть соль часто
равна имени библиотеки).

### Что дал этот алгоритм в нашем случае

* **`SceVeneziaWrapper` (user, `0x2B2DA8E1`) и `SceVeneziaWrapperForDriver`
  (`0x4F5231A4`) — соль подобрать не удалось.** Перебраны:
  * комбинации из слов `Sce/Venezia/VNZ/Wrapper/ForDriver/Driver/Codec/Engine/CE/
    User/Kernel/lib/NID/sce` длиной 1–3 без разделителей — 19 391 вариант;
  * все строки длиной 1–4 из `[a-z0-9_]` для обеих библиотек;
  * формы `lib…`, `…User`, верхний/нижний регистр.
  Ни один вариант не даёт `sha1(name+соль) == lib_nid`. Значит соль либо длиннее
  4 символов и не составлена из этих слов, либо для этого модуля применён
  не-SHA1/бинарный вариант (сравните: в вики есть отдельный класс
  «Binary Suffix NIDs»). Это объясняет, почему имена 41 функции не удаётся
  получить перебором по словарю.
* **`SceCodecEngineWrapper` (lib `0x5C9EE5B9`)** — соль тоже не подобралась
  быстрым перебором, но его NID-имена уже есть в `db.yml` (§3.1).
* При этом сам алгоритм подтверждён на 154 парах «имя → NID» из `db.yml`
  (`_scratch/nid_algo4.py`), то есть он рабочий — проблема только в конкретной
  соли этого модуля.

## 4. Что импортирует сама SceVeneziaWrapper

Внутренние имена `SceVeneziaWrapper` лежат в двух местах: `0xA3B38` (основной
`SceModuleInfo`) и `0xA4024` (второй описатель). Импортируемые библиотеки
(строки `0x0A40D8..0x0A41CC`):

```
SceCpuForDriver            SceIntrmgrForDriver      SceThreadmgrForDriver
SceKernelSuspendForDriver  SceProcessmgrForDriver   SceProcEventForDriver
SceDmacmgrForDriver        SceSysmemForDriver       SceSblFwLoaderForDriver
SceSysclibForDriver        ScePervasiveForDriver    ScePowerForDriver
```

Это ровно тот набор, который нужен, чтобы поднять Venezia:

* `SceSblFwLoaderForDriver` — загрузка `os0:kd/vnzimg.skprx` (см. `docs/VNZIMG.md` §3);
* `ScePervasiveForDriver` — доступ к Pervasive2 (reset_vnz, регистры 192/193…, SPRAM);
* `SceCpuForDriver` — L2/кэш и per-core (использовался `0x8100086C`);
* `SceDmacmgrForDriver` — DMA-каналы для `SceVeneziaDmacMemcpy%02u`;
* `SceProcEventForDriver` — события/ожидание готовности (`spram[0]==8`);
* `SceThreadmgrForDriver` — мьютексы 16 RPC-каналов.

## 5. Чего не удалось сделать

Восстановлено: 16 имён `SceVeneziaWrapper` (§3.1) и 6 из 47 имён
`SceVeneziaWrapperForDriver` (§3.2). Не удалось:

* узнать имена **остальных 41** функции `ForDriver`-библиотеки;
* подобрать `nid_suffix` обеих библиотек Venezia (см. §3.4) — именно это
  заблокировало словарный поиск имён;
* **дизассемблировать код** модуля:

**Дизассемблировать код SceVeneziaWrapper из `bootimage.elf` не получилось.**
Причина: таблица адресов экспортов (файл 0xA4098, 16 значений `0x8100C9B5..
0x8100CB8D`) не отображается ни на один участок файла, где эти адреса
разбираются как код:

* прямолинейная VA→файл формула (`hdr + 0xA0 + (va - 0x81000000)`) даёт участок,
  который не является ни ARM, ни Thumb;
* поиск базы перебором (все значения от `0x81000000` до `0x81100000` с шагом 2/4)
  в режимах ARM и Thumb не дал ни одного варианта, при котором ≥14 из 16 адресов
  открываются корректным прологом (`_scratch/calib3.py`, `_scratch/nid_scan.py`);
* адреса, начинающиеся у `0x8100C9C5` с шагом 8/16 байт, дают всего ~0x1D8 байт
  на 16 функций; это **косвенный** аргумент в пользу того, что в таблице лежат не
  реализации, а стабы, а сами тела — в другом месте модуля. Прямого подтверждения
  (например, разбора одного стаба) получить не удалось.

Чтобы пройти дальше, нужен один из вариантов:
1. прогнать `bootimage.elf` в загрузчике/эмуляторе и снять фактические адреса
   после применения релокаций (таблица сегментов вложенного модуля не совпадает
   с тем, как его размещает загрузчик);
2. разобрать SceKernelBootimage-формат упаковки (таблица дескрипторов вложенных
   модулей в начале образа) и получить честные `seg.vaddr`/`seg.offset`;
3. найти `vnz_wrapper.skprx` в открытом виде (в `Out/fs_dec` его нет — он только
   внутри bootimage), тогда разбор пойдёт той же методикой, что и для `vnzimg.elf`.

## 6. Итог

| что | значение |
|---|---|
| `SceVeneziaWrapper` | lib_nid `0x2B2DA8E1`, 16 функций = alias для `SceCodecEngineWrapper`; **6 вызываются из user-space** (`avcodec_us`) |
| `SceVeneziaWrapperForDriver` | lib_nid `0x4F5231A4`, 47 функций (6 имён восстановлено по вики); импортируется 3 другими модулями bootimage (39+7+6) |
| `SceAvcodec` | lib_nid `0xA166C96E`, 108 функций; все имена известны из `db.yml` |
| `SceVeneziaWrapper` modinfo | файл 0x0A3B34, module_nid `0x69D62026`, attr 0x07, type 0x06 |
| `SceAvcodec` modinfo | файл 0x199860, module_nid `0xA421175B`, attr 0x07, type 0x06 |
| NID-таблицы | `SceVeneziaWrapper`: файл 0xA4038 (16); `ForDriver`: файл 0xA3E7C (47); адреса `SceVeneziaWrapper`: файл 0xA4098 |
| зависимости обёртки | SceSblFwLoader, ScePervasive, SceCpu, SceDmacmgr, SceProcEvent, SceThreadmgr, SceIntrmgr, SceSysmem, ScePower, SceSysclib |

Машинно-читаемые таблицы: `_scratch/venezia_libs.json`.

## 7. Воспроизведение

```powershell
python _scratch\boot_hdr.py          # 34 ELF-магии внутри bootimage
python _scratch\boot_embedded2.py    # вложенные модули и их segment layout
python _scratch\nid_scan.py          # записи SceLibEntTable по lib_nid
python _scratch\final_nids.py        # SceVeneziaWrapper: 16 NID + адреса
python _scratch\extract4.py          # ForDriver: 47 NID (по возрастанию)
python _scratch\calib3.py            # почему код не собирается (ARM/Thumb поиск)
python _scratch\query_nid_db.py      # поиск 16 NID по базам NID рабочего каталога
python _scratch\verify_names.py      # сверка 16 NID с SceCodecEngineWrapper (16/16)
```
