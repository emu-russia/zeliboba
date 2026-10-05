# Полный стек использования Venezia/VIP: от SDK-демок до MeP

Прошивка 1.04 + SDK 0.945.040. Разбор отвечает на вопрос: как вызов вида
`sceAudiodecDecode()` или `sceJpegDecodeMJpegYCbCr()` в демке из SDK превращается
в работу на сопроцессорах Venezia/VIP и как результат возвращается.

Источники: SDK-сэмплы
`Sony_Vita_Sdk_0945-YLoD/InstallFiles/[26]/sdk/target/samples/sample_code/audio_video`,
SDK-заголовки и stub-библиотеки там же, документация SDK (`Audio_Video/*.pdf`),
decrypted-модули прошивки и разборы `docs/VNZIMG.md`, `docs/VIPIMG.md`.

---

## 1. Слои

```
 SDK-демка (audio_video/api_*)
   |  sceAudiodecDecode / sceJpegDecodeMJpegYCbCr / sceAvcdecDecode / sceSas*
   v
 линкуется libSceAudiodec_stub.a / libSceJpeg_stub.a / ...
   |  импорт-таблица: <библиотека, NID>            [проверено: libSceAudiodec_stub.a
   v                                                  -> SceAudiodecUser 0x2AA62046]
 user-модуль  os0/us/avcodec_us.elf  (SceAvcodecUser, module NID 0x88EB00C1)
   |  экспортирует 6 библиотек:
   |    SceAudiodecUser  0x2AA62046  (12 функций)
   |    SceAudioencUser  0xA938B1A6  (13)
   |    SceJpegUser      0x880BF710  (11)
   |    SceJpegEncUser   0x3748BFD3  (10)
   |    SceVideodecUser  0x163C3727  (42)
   |    SceVideoencUser  0x0BAA6DEF  (20)
   |    SceCodecEngineUser 0x469AB062 (6)
   |  импортирует:
   |    SceAvcodec          0xA166C96E  (108 функций)
   |    SceVeneziaWrapper   0x2B2DA8E1  (6 функций)   <- выход на Venezia
   v
 kernel: bootimage.elf (SceKernelBootimage) содержит встроенные модули
   |    SceVeneziaWrapper / SceVeneziaWrapperForDriver   -> грузит vnzimg.skprx
   |    SceVipDriver / SceVip                            -> грузит vipimg.skprx
   |    SceAvcodec, SceCodecEngine
   v
 образы MeP:  vnzimg.elf (SceVeneziaImage, 1 010 168 Б, VLIW/IVC2)
              vipimg.elf (SceVipImage,      360 088 Б, scalar, без VLIW)
```

Проверено скриптами: импорт-таблицы (`_scratch/imp_raw.py`, `_scratch/chain.py`),
поиск импортёров (`_scratch/who_imports.py`), состав библиотек
(`_scratch/modtables.py`, `_scratch/db_nids.py`).

## 2. Что говорит SDK: у каждого кодека есть «внешний» вариант

В `db.yml` у `SceAvcodec` (kernel NID `0xA166C96E`) для аудио, изображений и видео
есть **по три варианта одной операции**:

| обычный | External | Resident |
|---|---|---|
| `_sceAudiodecCreateDecoder` | `_sceAudiodecCreateDecoderExternal` | `_sceAudiodecCreateDecoderResident` |
| `_sceAudiodecDecode` | — | — |
| `_sceAudioencCreateEncoder` | `_sceAudioencCreateEncoderExternal` | `_sceAudioencCreateEncoderResident` |
| `_sceAvcdecCreateDecoder` | `_sceAvcdecCreateDecoderInternal` | `_sceAvcdecCreateDecoderNongameapp` |

`External` / `Resident` — это и есть выбор между встроенным (software) декодером и
**внешним кодечным движком**.

В `bootimage.elf` рядом с `SceAvcodec` (`0x199864`) видны и другие имена той же
семьи: **`SceAvcodecForDriver`** (`0x199ABC`), а также `SceAudiodecResident`,
`SceAudiodec`, `SceAudioencResident` (`0x19A5A4`). То есть деление
software / external / resident зашито и в имена библиотек прошивки, а не только
в NID. В user-API то же деление повторяется:
`sceAudiodecCreateDecoder` / `sceAudiodecCreateDecoderExternal` /
`sceAudiodecCreateDecoderResident`.

Документация SDK формулирует это без имён железа:

* **libaudiodec**: «a library for decoding audio data **with a codec engine**»;
  кодеки ATRAC9™, MP3 (MPEG1/2/2.5 Layer3), AAC (LC, HE-AAC v1/v2), CELP.
* **scejpegdec**: «Supports high-speed color space conversion functions **using
  dedicated hardware** (256-level YCbCr, YCbCr422/420 → RGBA)», downscale 1/2, 1/4, 1/8.
* **avcdec**: «**Dedicated hardware** is used to perform AVC decoding and color
  space conversion (YCbCr → RGBA)», до 960x544.

Слов «Venezia»/«VIP» в SDK-документации нет — SDK абстрагируется до «codec
engine» и «dedicated hardware». Имена железа видны только в прошивке.

## 3. Путь изображений (JPEG)

Демка `api_scejpeg/simple/jpegdec.c` (проверено построчно):

```c
sceJpegInitMJpeg(0);
sceJpegGetOutputInfo(pJpeg, isize, SCE_JPEG_NO_CSC_OUTPUT, decodeMode, &outputInfo);
//   -> imageWidth/Height, outputBufferSize, coefBufferSize, colorSpace
decodeMode = SCE_JPEG_MJPEG_WITH_DHT | SCE_JPEG_MJPEG_DOWNSCALE_1_2|1_4|1_8;
sceJpegDecodeMJpegYCbCr(pJpeg, isize, pYCbCr, decodeBufSize, decodeMode,
                        pCoefBuffer, coefBufSize);          // -> YCbCr, ret = W<<16|H
sceJpegMJpegCsc(pFrame, pYCbCr, ret, width, SCE_JPEG_PIXEL_RGBA8888, colorSpace);
```

Ключевая деталь по буферам — из демки:

```c
/* Allocate buffers from CDRAM.
   Because the JPEG decoder needs consecutive areas in a physical address. */
bufferMemBlock = sceKernelAllocMemBlock("jpegdecBuffer",
    SCE_KERNEL_MEMBLOCK_TYPE_USER_CDRAM_RWDATA, totalBufSize, SCE_NULL);
```
`totalBufSize` округляется вверх до 256 КиБ, а внутри — три области, выровненные
по 256 байт: поток, YCbCr-выход, буфер коэффициентов ДКП.

**Что это значит:** требование непрерывных *физических* адресов в CDRAM —
признак DMA в блок, который читает физическую память напрямую. Отсюда и цепочка:

`SceJpegUser` (в `avcodec_us`) → `SceAvcodec` (kernel, bootimage) →
аппаратный JPEG-блок. По именам модулей прошивки этот аппаратный путь —
**VIP**: `SceVipDriver`, `SceVipMailbox0`, `SceVipDma`, `SceVipDmacMemcpy%02u`,
`SceVipLpddrCode`, `SceVipSuspend` (строки в `bootimage.elf`
`0xA4644–0xA46A0`), а сам образ — `vipimg.elf` (`SceVipImage`).

Отдельная функция `sceJpegCsc` (`0x6263AEC2`, есть в `SceJpegUser`) — это,
по документации, «high-speed color space conversion using dedicated hardware»,
то есть CSC тоже уходит в блок, а не считается на ARM.

> Оговорка: импорт `SceVip*` в `avcodec_us` **не найден** (проверено
> `who_imports.py`: `avcodec_us` импортирует только `SceAvcodec` и
> `SceVeneziaWrapper`). Значит, к VIP из user-space ходят через kernel-модуль
> `SceAvcodec`/`SceVipDriver`, а не напрямую. Точную развилку
> «software vs VIP» внутри `SceAvcodec` я не дизассемблировал — это открытый
> пункт (§8).

## 4. Путь аудио (ATRAC9 / MP3 / AAC / CELP)

Демка `api_libaudiodec/on_memory` (проверено): общий декодер выбирает
инициализацию по типу кодека и на каждый кадр делает один вызов.

```c
// audio_decoder.c
switch (codecType) {
case SCE_AUDIODEC_TYPE_AT9:  initAudioDecoderAt9 (pCtrl, pInput, pAudioOut); break;
case SCE_AUDIODEC_TYPE_MP3:  initAudioDecoderMp3 (pCtrl, pInput, pAudioOut); break;
case SCE_AUDIODEC_TYPE_AAC:  initAudioDecoderAac (pCtrl, pInput, pAudioOut); break;
case SCE_AUDIODEC_TYPE_CELP: initAudioDecoderCelp(pCtrl, pInput, pAudioOut); break;
}

int decodeAudio(...) {
    pCtrl->pEs  = pInput->p  + pInput->offsetR;   // вход: elementary stream
    pCtrl->pPcm = pOutput->p + pOutput->offsetW;  // выход: PCM
    res = sceAudiodecDecode(pCtrl);               // <-- одна операция
    pInput->offsetR  += pCtrl->inputEsSize;
    pOutput->offsetW   = (pOutput->offsetW + pOutput->size / DOUBLE_BUFFER)
                         % pOutput->size;
}
```

То есть API предельно простой: **вход — указатель на ES, выход — указатель на
PCM**, а `SceAudiodecCtrl` несёт тип кодека, параметры и размеры. Всё остальное
(где реально считаются сэмплы) скрыто за `sceAudiodecDecode`.

Типы (из `audiodec.h`):

| кодек | тип | каналы | частоты | макс. сэмплов | макс. ES |
|---|---|---|---|---|---|
| ATRAC9™ | `0x1003` | 1–2 (в библиотеке до 16) | 48000 | 256 | 1024 |
| MP3 | `0x1004` | 1–2 | 8–48 кГц | 1152 | 1441 |
| AAC | `0x1005` | 1–2 | 8–48 кГц | — | — |
| CELP | `0x1006` | 1 | 8 кГц (в энкодере) | — | — |

Потребители в user-space: `avcodec_us` (основной), `libvoice.elf`
(`SceAudiodecUser`, 5 функций), `initialsetup.elf` (4 функции), `libsas.elf`
(звуковой синтез, отдельная ветка). Демки: `api_libaudiodec`,
`api_libcodecengine` (тот же декодер, но через `SceCodecEngine`),
`api_libsas`, `api_libngs`.

## 5. Куда именно уходит работа: Venezia

Единственная **прямая** связь user-space с Venezia — это импорт
`SceVeneziaWrapper` (lib NID `0x2B2DA8E1`) в `avcodec_us`:

```
avcodec_us.elf, import entry @0xD5C:
  size=0x34, num_functions=6
  lib_nid    = 0x2B2DA8E1  -> "SceVeneziaWrapper"
  nid_table  = 0x8100122C
  entry_table= 0x810013F4  -> 0x810004DC, 0x810004EC, 0x810004FC,
                              0x8100050C, 0x8100051C, 0x8100052C
  NID: 0x03DCBDCA, 0x04BA9415, 0x362E9415, 0x489FF965, 0xAD30912D, 0xB0E654EE
```

С другой стороны, в `bootimage.elf` встроен модуль, экспортирующий
`SceVeneziaWrapper` / `SceVeneziaWrapperForDriver` — он же грузит
`os0:kd/vnzimg.skprx` и ведёт протокол подъёма Venezia
(см. `docs/VNZIMG.md` §3: Pervasive2 `regs[192..196]/[224..239]/[256]/[898]`,
`spram[0]=1`, `ScePervasiveForDriver 0xA7E64C6F` = reset_vnz, ожидание
`spram[0]==8`). Проверено в `bootimage.elf`: `SceModuleInfo` этого модуля лежит
по файловому смещению `0xA3B34` (name field `0xA3B38`), `attr=0x07`, `type=0x06`,
`gp=0`, диапазон экспорта `0xCBFC..0xCC5C` (то есть **две** записи библиотек по
0x20 байт). Разбор самих имён функций и их числа требует отдельного прохода по
внутренней таблице сегментов модуля — здесь этот пункт оставлен открытым (§8).

Имена, которые встречаются в окружении модуля: `SceVeneziaRpcCh`,
`SceVeneziaSpram`, `SceVeneziaRpc`, `SceVeneziaRpcDebug`,
`SceVeneziaRpcBlocking`, `SceVeneziaMailbox%02u`,
`SceVeneziaRpcCommunicationHeap`, `SceVeneziaRpcDma`,
`SceVeneziaDmacMemcpy%02u`, `SceVeneziaSuspend`, `ScePervasive2Reg`
(смещения `0xA454C–0xA45F0`).

**Смысл шести функций.** Их ровно шесть на весь user-space, и они не похожи на
108 функций `SceAvcodec` (которые покрывают создание/удаление/очереди
декодеров). Шесть — это, по совокупности признаков (пары `Create/Delete`,
`Decode`, и то, что это единственный канал наружу), интерфейс
**аппаратного кодека**:

* инициализация/открытие канала к MeP-движку,
* отправка задания (вход: ES-буфер, выход: PCM/YUV-буфер),
* ожидание/приём результата (RPC-канал + событие),
* закрытие канала.

Прямое подтверждение имён NID в `db.yml` отсутствует (эта библиотека там не
расписана, таблица 3.60), поэтому привязка NID↔операция — вывод, а не факт.
Косвенно её поддерживает состав `SceVeneziaWrapper` (16 функций) в прошивке:
там есть и `SceVeneziaRpc*`.

**Что уходит в Venezia и что возвращается** (по протоколу из `docs/VNZIMG.md`):

* **уходит**: 256-байтовый командный блок в SPRAM
  (`memcpy(spram, 0x8100ED8C, 0x100)` по `0x81001034`), плюс сами данные
  (ES/JPEG-поток) в разделяемой памяти;
* **синхронизация**: ARM пишет `spram[0]=1`, ждёт `spram[0]==8`; далее работают
  16 RPC-каналов `SceVeneziaRpc*` (`SceVeneziaRpcCh`, `SceVeneziaRpc`,
  `SceVeneziaRpcDebug`, `SceVeneziaMailbox%02u`, `SceVeneziaDmacMemcpy%02u`),
  у каждого своё событие и регистр статуса `0x8100EE8C+0x10·n`;
* **возвращается**: результат в общем буфере (PCM для аудио, YCbCr/RGBA для
  изображений), признак готовности — через mailbox/событие RPC-канала.

### 5.1 `SceCodecEngine` — это монитор загрузки сопроцессоров

`avcodec_us` экспортирует ещё `SceCodecEngineUser` (`0x469AB062`, 6 функций), и
SDK-заголовок `codecengine.h` объясняет, что это такое:

```c
typedef struct SceCodecEnginePmonProcessorLoad {
    SceUInt32 size;
    SceUInt32 average;
} SceCodecEnginePmonProcessorLoad;

sceCodecEnginePmonStart(void);
sceCodecEnginePmonStop(void);
sceCodecEnginePmonGetProcessorLoad(&load);
sceCodecEnginePmonReset(void);
```

То есть SDK даёт приложению **измеритель загрузки кодечного движка** — прямое
подтверждение, что декодирование выполняет отдельный процессор (а не ARM):
иначе измерять было бы нечего. В прошивке этому соответствуют
`SceCodecEngineBoot`, `SceCodecEngineProcessor` и
`SceCodecEnginePerformanceMonitor` (строки в `bootimage.elf`).

## 6. Сводная матрица

| что декодируется | user-API (SDK) | библиотека (user) | за кулисами |
|---|---|---|---|
| JPEG | `sceJpegDecodeMJpegYCbCr`, `sceJpegCsc` | `SceJpegUser` (в `avcodec_us`) | `SceAvcodec` → аппаратный путь; по именам модулей — **VIP** (`SceVipDriver` + `vipimg.elf`) |
| AVC/H.264 | `sceAvcdecDecode` | `SceVideodecUser` | «dedicated hardware» (SDK), внутренние варианты `_sceAvcdec*Internal` |
| ATRAC9 | `sceAudiodecDecode` + `SCE_AUDIODEC_TYPE_AT9` | `SceAudiodecUser` | `SceAvcodec` → **Venezia** через `SceVeneziaWrapper` (6 импортируемых функций) |
| MP3 | то же, `TYPE_MP3` | `SceAudiodecUser` | то же |
| AAC | то же, `TYPE_AAC` | `SceAudiodecUser` | то же |
| CELP | то же, `TYPE_CELP` | `SceAudiodecUser` | то же |
| звук в игре (синтез) | `sceSas*`, `sceNgs*` | `libsas.elf`, `libngs` | отдельная ветка, DSP-эффекты |

Про ATRAC3: в SDK **нет** отдельного кодека ATRAC3 — есть **ATRAC9**
(`SCE_AUDIODEC_TYPE_AT9`, `ATRAC9-Format_e.pdf`, утилита `at9tool`).
ATRAC3 как отдельный тип в заголовках 0.945 не встречается; исторически это
кодек PSP/ATRAC3plus, на PSVita аудио идёт через ATRAC9/AAC/MP3/CELP.
`SCE_AUDIOENC_CELP_MPE` в `audioenc.h` — это **не** движок Venezia, а режим
возбуждения CELP (Multi-Pulse Excitation), константа `0`.

## 7. Что это даёт для эмулятора

Порядок, в котором имеет смысл реализовывать:

1. **VIP-путь для изображений** — самый простой: образ `vipimg` scalar, без
   VLIW, обмен через control bus. Достаточно отобразить образ, завести окно
   control bus и mailbox (`SceVipMailbox0`, `SceVipDma`).
2. **Venezia-путь для аудио** — сложнее: нужны IVC2/VLIW (в образе 1625
   переключений), SPRAM `0xF1840000`, 256-байтовый командный блок и 16 RPC-каналов.
3. **Стык user↔kernel**: `avcodec_us` — тонкий слой (сегмент 0x1E00, из них
   код до 0x1DC0); он вызывает 108 функций `SceAvcodec` и 6 функций
   `SceVeneziaWrapper`. Для эмуляции достаточно заглушить эти два интерфейса,
   не поднимая весь SDK.

## 7.1 Связанные разборы

* `docs/VNZIMG.md` — контейнер `vnzimg`, протокол подъёма Venezia, RPC, SPRAM;
* `docs/VNZIMG_MEP.md` — дизассемблер образа MeP (вектора, инициализация,
  1625 переключений в VLIW);
* `docs/VIPIMG.md` — модуль-близнец `vipimg` (VIP, без VLIW);
* `docs/VENEZIA.md` — сводка по Venezia с поправками.

## 7.2 NID-таблицы обёртки и SceAvcodec

Восстановлены NID и состав `SceVeneziaWrapper` (16 функций, lib_nid
`0x2B2DA8E1`, из них 6 вызывает user-space), `SceVeneziaWrapperForDriver`
(47 функций, `0x4F5231A4`) и `SceAvcodec` (108 функций, `0xA166C96E`) —
**`docs/VENEZIA_WRAPPER_AVCODEC.md`**.

## 8. Открытые пункты

1. ~~Имена шести импортируемых NID `SceVeneziaWrapper`~~ — **решено**: это
   `_sceCodecEngine*`; `SceVeneziaWrapper` (lib `0x2B2DA8E1`, 16 функций) —
   alias для `SceCodecEngineWrapper` (lib `0x5C9EE5B9`, те же 16). Таблица
   имён — `docs/VENEZIA_WRAPPER_AVCODEC.md` §3.1. По `SceVeneziaWrapperForDriver` (47 функций) найдено 6 имён
   (вики); алгоритм NID проверен, соль библиотеки не подобрана. Осталось:
   остальные 41 имя и дизассемблер самого кода обёртки.
2. Точная развилка «software или аппаратный декодер» внутри `SceAvcodec`:
   сравнить `_sceAudiodecDecode` и `_sceAudiodecDecodeExternal` (второй в
   заголовках не экспортируется, но есть в `SceAvcodec`).
3. Действительно ли JPEG/AVC идут через VIP, а не через отдельный блок:
   проверить, вызывает ли `SceAvcodec` `SceVipDriver` (в `avcodec_us` импорта
   `SceVip*` нет — значит связь внутри kernel).
4. Формат команд VIP (mailbox/DMA) — из `SceVipDriver` в `bootimage.elf`.
5. Есть ли на PSVita **ATRAC3** вообще: в SDK 0.945 — только ATRAC9.

## 9. Воспроизведение

```powershell
# SDK: сэмплы и заголовки
Get-ChildItem -LiteralPath "C:\Work\PSVita\Sony_Vita_Sdk_0945-YLoD\InstallFiles\[26]\sdk\target\samples\sample_code\audio_video"
Get-Content -LiteralPath "C:\Work\PSVita\Sony_Vita_Sdk_0945-YLoD\InstallFiles\[26]\sdk\target\include\audiodec.h"

# прошивка: таблицы модулей
python _scratch\modtables.py  <module.elf>
python _scratch\imp_raw.py                 # импорт-записи avcodec_us
python _scratch\who_imports.py             # кто импортирует SceVip*/SceVeneziaWrapper
python _scratch\db_nids.py                 # имена NID из db.yml
python _scratch\sdk_stubs.py               # состав stub-библиотек SDK
python _scratch\sdk_doc_ctx.py             # выдержки из PDF SDK
```
