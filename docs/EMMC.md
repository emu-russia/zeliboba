# eMMC image reconstruction (firmware 1.04)

The console we are bringing up has **no eMMC dump**, so the card is rebuilt out of
the 1.04 firmware that we do have. This document records exactly what is written
where, the evidence for every layout decision, and what could not be verified.

Everything described here is implemented in `src/hw/emmc/` and driven by
`tools/emmc_rebuild.cpp`:

| file | contents |
| --- | --- |
| `src/hw/emmc/emmc_card.cpp` | `EmmcCard`: file backed card, partitions, CID/CSD/EXT_CSD, read/write/erase |
| `src/hw/emmc/emmc_fat.cpp` | FAT16 volume builder and reader/verifier |
| `src/hw/emmc/emmc_image.cpp` | `build_emmc_image`, `inspect_emmc_image`, `verify_emmc_image`, SLB2 + master block |
| `src/hw/emmc/emmc.h` | internal constants and helpers |
| `src/hw/emmc.h` | the public interface (`EmmcCard`, `EmmcImagePlan`, ...) |
| `tools/emmc_rebuild.cpp` | CLI: `--firmware --out --size --inspect --verify --from-tree --verbose` |
| `tests/test_emmc.cpp` | unit tests (geometry, partitions, registers, SLB2, FAT) |

```
emmc_rebuild --firmware ..\Vita_104_Firmware\Out --out build\emmc.img          # full 3.55 GiB user area
emmc_rebuild --firmware ..\Vita_104_Firmware\Out --out build\test.img --size 512M
emmc_rebuild --inspect build\emmc.img
emmc_rebuild --verify  build\emmc.img --firmware ..\Vita_104_Firmware\Out
emmc_rebuild --firmware ..\Vita_104_Firmware\Out --out build\tree.img --from-tree
```

---

## 1. Card geometry (Toshiba THGBM3G5D1FBAIE, 4 GiB, eMMC 4.41)

Evidence: `datasheets/thgbm3g5d1fbaie32nm4gbe-mmc_e_rev0.3_100917.pdf`
(pages 4-9, "Density Specifications", "CSD Register", "Extended CSD Register").

| item | value | source |
| --- | --- | --- |
| User area (datasheet) | 3,997,171,712 bytes = 0xEE3C0000, SEC_COUNT 0x00772000 | datasheet page 4 |
| User area (console's own table) | 0x71A000 blocks = 3.55 GiB | reference eMMC dump, master block |
| Boot partitions | 2 x 2 MiB (BOOT_SIZE_MULTI = 0x10 x 128 KiB) | see note below |
| RPMB | 512 KiB in the model (`kRpmbSize`), i.e. `RPMB_SIZE_MULT = 0x01` x 128 KiB **rounded up** — an assumption, not a datasheet number | `src/hw/emmc/emmc.h` |
| Block size | 512 bytes | READ_BL_LEN / WRITE_BL_LEN = 0x9 |
| CID MID | 0x11 (Toshiba) | datasheet page 4 |
| CID OID | 0x00 | datasheet page 4 |
| CID CBX | 01b (BGA) | datasheet page 4 |
| CID PRV / PSN / MDT | 0x03 / 0x5A1B0B00 / MDT = 0x30 (the model's u16 field is 0x0130; the CID field itself is 8 bits) | placeholder, the real serial is per console |
| CSD_STRUCTURE / SPEC_VERS | 3 / 4 (CSD version 1.0, high capacity) | datasheet page 5 |
| CSD TAAC / NSAC / TRAN_SPEED | 0x0E / 0x00 / 0x32 (26 MHz) | datasheet page 5 |
| CSD CCC | 0x0F5 | datasheet page 5 |
| CSD C_SIZE | computed so that `(C_SIZE + 1) * 512 KiB == user area` | CSD 1.0 formula |
| CSD ERASE_GRP_SIZE / MULT, WP_GRP_SIZE | 0x1F / 0x1F / 0x03 | datasheet page 5 |
| EXT_CSD BOOT_INFO / BOOT_SIZE_MULTI | 0x07 / 0x10 | datasheet page 6 (0x04 is the preliminary value) |
| EXT_CSD ACC_SIZE / HC_ERASE_GRP_SIZE / ERASE_TIMEOUT_MULT | 0x06 / 0x04 / 0x02 | datasheet page 6 |
| EXT_CSD REL_WR_SEC_C / HC_WP_GRP_SIZE / S_A_TIMEOUT | 0x10 / 0x01 / 0x10 | datasheet page 6 |
| EXT_CSD SEC_COUNT | 0x71A000 (the user area we build) | computed |
| EXT_CSD RPMB_SIZE_MULT | 0x01 | datasheet page 7 |
| EXT_CSD PARTITIONING_SUPPORT | 0x03 | datasheet page 7 |
| EXT_CSD WR_REL_PARAM | 0x05 | datasheet page 7 |
| EXT_CSD EXT_CSD_REV | 0x02 (1.5.1) | eMMC 4.41 parts |

The image file is laid out as

```
[ user area ][ boot0 2 MiB ][ boot1 2 MiB ][ RPMB 512 KiB ]
 0x00000000   0xE3400000      0xE3600000     0xE3800000     (for the default size)
```

so a 4 GiB file describes the whole part. Reads and writes go straight to the
file with 64 bit seeks; nothing is cached in RAM. Images are created **sparse**
(`FSCTL_SET_SPARSE`), so a 3.56 GiB image that only contains ~100 MiB of firmware
costs only the bytes that are actually written.

**Assumptions / discrepancies**

* The card's CID serial number and manufacturing date are placeholders
  (`0x5A1B0B00`, 2011-03); the real ones are per console and are not in the PUP.
* The datasheet's SEC_COUNT (3,997,171,712 bytes) is ~184 MiB larger than the
  0x71A000 blocks the console's own partition table declares. The reconstructed
  image uses the console's figure so that the tables the tools print agree with
  the hardware; the datasheet value is kept as
  `emmc::kPartDatasheetUserBytes` for reference.
* The datasheet quotes `BOOT_SIZE_MULTI = 0x04` (512 KiB) while the console's
  partition table reserves 2 MiB per boot partition (two SLB2 partitions of
  0x2000 blocks each). The 1.04 console value (0x10) is used.
* EXT_CSD fields that the firmware does not appear to consume (power class
  values, min-performance values, BKOPS) are set to the datasheet defaults or
  left zero.

---

## 2. Partition table (LBA 0 of the user area)

The Vita does **not** use a PC MBR. LBA 0 carries a Sony "master block":

```
0x000  char[32] "Sony Computer Entertainment Inc."
0x020  u32      version = 3
0x024  u32      total user area in 512 byte blocks (0x71A000)
0x028  u32      0
0x02C  u32      0
0x030  u32      0x0000606A, 0x00000069, 0x00006000, 0x00004000, 0x00006000, 0x00010000
0x048  u32      0
0x050  records, 17 bytes each, 16 slots:
         +0x00 u32 offset in 512 byte blocks
         +0x04 u32 size   in 512 byte blocks
         +0x08 u8  partition code
         +0x09 u8  partition type
         +0x0A u8  active flag
         +0x0B u32 flags
         +0x0F u8  0x0F (constant on the console)
0x1FE  u16 0xAA55
```

Evidence: the partitioned eMMC dump in `dumps/emmcdump.zip`
(`emmcdump.bin`, 25,165,824 bytes = the first 24 MiB of the user area). Its first
sector has the magic string, version 3, the 0xAA55 signature and a 16 entry
record table starting at 0x50 with a **17 byte** stride - decoding it with that
stride (and not 16) is what makes every field land on a sensible value, and it
is what produces the offsets 8 MiB / 12 MiB / 16 MiB that the dump shows
`SLB2`, `SLB2` and `OS0` living at. Note that this dump is a **format reference
only**: it comes from a different console and is not used as data. The CP
(syscon) of that console is not emulated either; nothing in the image depends on
it.

The reproduced 1.04 table (the builder writes this, and only declares partitions
that fit inside the image being created):

| # | name | code | type | offset | size | active | flags | payload we write |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | idstorage | 0x01 IDSTORAGE | 0xDA RAW | 0x00040000 | 0x00080000 (512 KiB) | 0 | 0x001F0F00 | reconstructed leaf index (see §5) |
| 1 | bls0 | 0x02 SLB2 | 0xDA RAW | 0x00800000 (8 MiB) | 0x00400000 (4 MiB) | 0 | 0x000F0F00 | SLB2 container |
| 2 | bls1 | 0x02 SLB2 | 0xDA RAW | 0x00C00000 (12 MiB) | 0x00400000 (4 MiB) | 1 | 0x000F0F01 | SLB2 container (backup) |
| 3 | os0_0 | 0x03 OS0 | 0x06 FAT16 | 0x01000000 (16 MiB) | 0x01000000 (16 MiB) | 0 | 0x000F0F00 | real `os0.bin` |
| 4 | os0_1 | 0x03 OS0 | 0x06 FAT16 | 0x02000000 (32 MiB) | 0x01000000 (16 MiB) | 1 | 0x000F0F01 | real `os0.bin` (backup) |
| 5 | sa0 | 0x0C SA0 | 0x06 FAT16 | 0x03000000 (48 MiB) | 0x06000000 (96 MiB) | 0 | 0x000F0F00 | erased |
| 6 | tm0 | 0x06 TM0 | 0x06 FAT16 | 0x09000000 (144 MiB) | 0x02000000 (32 MiB) | 0 | 0x000F0F00 | erased |
| 7 | vs0_0 | 0x04 VS0 | 0x06 FAT16 | 0x0B000000 (176 MiB) | 0x10000000 (256 MiB) | 0 | 0x000F0F00 | real `vs0.bin` |
| 8 | vd0 | 0x05 VD0 | 0x06 FAT16 | 0x1B000000 (432 MiB) | 0x02000000 (32 MiB) | 0 | 0x000F0F00 | erased |
| 9 | ud0 | 0x0B UD0 | 0x06 FAT16 | 0x1D000000 (464 MiB) | 0x10000000 (256 MiB) | 0 | 0x000F0F00 | erased |
| 10 | pd0 | 0x0E PD0 | 0x07 EXFAT | 0x2D000000 (720 MiB) | 0x13000000 (304 MiB) | 0 | 0x000F0F00 | erased (exFAT on hardware) |
| 11 | ur0 | 0x07 UR0 | 0x07 EXFAT | 0x40000000 (1 GiB) | 0x0A340000 (2612 MiB) | 0 | 0x000F0F00 | erased (exFAT on hardware) |

The regions add up to exactly the user area the master block declares
(0x71A000 blocks). Partitions whose end lies beyond the image being created are
not declared at all, so a small test image still reports a consistent table
(`--size 512M` yields the first ten records).

**Assumption:** only `os0`, `vs0`, the two SLB2 copies and `idstorage` carry
content; the PUP has nothing for sa0/tm0/vd0/ud0/pd0/ur0, so those are left
untouched inside the sparse image and reported as "erased".

---

## 3. Boot stage: the SLB2 container

The 1.04 container was recovered from the PUP itself
(`Out/PUP_dec/boot_slb2-00.pkg.seg02`, 647,168 bytes) and cross-checked against
the two SLB2 copies in the eMMC dump. Layout:

```
0x00  char[4] "SLB2"
0x04  u32     version = 1
0x08  u32     entry table size = 0x200
0x0C  u32     entry count
0x10  u32     data offset hint = 0x2000
0x14  u32     reserved (12 reserved bytes, 0x14..0x1F)
0x20  entries, 0x30 bytes each, count of them:
        +0x00 u32 first 512 byte block of the payload
        +0x04 u32 payload size in bytes
        +0x08 u32 flags
        +0x0C u32 reserved
        +0x10 char[32] name
      unused table bytes (up to 0x200) are filled with 0xFF
      every name field is NUL padded (the boot chain reads it as a C string)
then  the payloads, each aligned to a 512 byte block
```

The 1.04 entries and where their magics land (all verified against the file):

| # | name | first block | byte offset | size | magic |
| --- | --- | --- | --- | --- | --- |
| 0 | second_loader.enp | 0x01 | 0x200 | 93,184 | MeP header 0x64B2C8E5 |
| 1 | second_loader.enc | 0xB7 | 0x16E00 | 93,184 | MeP header 0x64B2C8E5 |
| 2 | secure_kernel.enp | 0x16D | 0x2DA00 | 33,280 | MeP header 0x64B2C8E5 |
| 3 | secure_kernel.enc | 0x1AE | 0x35C00 | 33,280 | MeP header 0x64B2C8E5 |
| 4 | kernel_boot_loader.self | 0x1EF | 0x3DE00 | 355,220 | SCE header "SCE\0" |
| 5 | kprx_auth_sm.self | 0x4A5 | 0x94A00 | 35,064 | SCE header "SCE\0" |
| 6 | prog_rvk.srvk | 0x4EA | 0x9D400 | 1,728 | SCE header "SCE\0" |

The builder slices these seven payloads straight out of the retail container, so
the assembled table, the names and the 7 payloads are byte identical to the PUP
(verified: the only difference inside the first 0xA000 bytes is 12 reserved
header bytes at 0x14..0x1F).

The container is written to **four** places: the two eMMC boot partitions
(`EmmcPartition::Boot0`/`Boot1`, file offsets 0xE3400000/0xE3600000 for the
default size) and the two user area SLB2 partitions at 8 MiB and 12 MiB. The
machine's boot chain looks for the container in the boot partitions first and
only then in the user area (`read_slb2_container()`, `src/machine/bootchain.cpp`);
the user area copies are the fallback.

---

## 4. os0 and vs0: the real 1.04 FAT16 partition images

`Out/PUP_dec/os0.bin` (6,594,560 bytes) and `Out/PUP_dec/vs0.bin`
(87,445,504 bytes) **are** the filesystems the console has in those partitions;
they are what the PUP ships. The default build therefore lays them down
**verbatim** at 0x01000000 (os0_0), 0x02000000 (os0_1) and 0x0B000000 (vs0_0) and
zero fills the rest of each slot.

This was verified to be lossless before it was made the default: walking every
cluster chain in both images shows that the last byte referenced by any file is
0x65DAC0 (os0) and 0x5422389 (vs0), both inside the extracted images, so **all
992 files of the two volumes are completely present**. The extracted `fs/` tree
that ships next to them has the same 63 files / 6,405,765 bytes and 929 files /
84,375,093 bytes, and `--verify` compares the two byte for byte (0 mismatches).

Geometry, read out of the real images and reused by `--from-tree`:

| | os0 | vs0 |
| --- | --- | --- |
| bytes per sector | 512 | 512 |
| sectors per cluster | 8 (4 KiB) | 8 (4 KiB) |
| reserved sectors | 2 | 2 |
| FATs | 2 | 2 |
| root entries | 512 | 512 |
| sectors per FAT | 19 | 259 |
| total sectors | 0x8000 (16 MiB) | 0x20000... 0x80000 (256 MiB) |
| media descriptor | 0xF8 | 0xF8 |
| OEM | "SCEI" | "SCEI" |
| label | "NO NAME" | "NO NAME" |
| volume id | 0x3F5A3D45 | 0x3F5A3D45 |
| clusters | 4087 | 65467 |
| free clusters | 2487 | 44188 |

`--from-tree` synthesises the same volumes from `Out/fs/os0` and `Out/fs/vs0`
instead, using exactly that geometry (boot sector, FSInfo, both FAT copies, the
fixed 512 entry root, directories with `.`/`..`, LFN entries for names that do
not fit 8.3, and the cluster chains). `--verify` passes for that mode too.

FAT16 rather than exFAT is what the console actually uses here (`type = 0x06` in
the master block for both partitions), and both kernels ship the driver
(`Out/fs/os0/kd/exfatfs.skprx` and `fatfs` in the boot image). Writing the real
images keeps the choice identical to hardware.

---

## 5. idstorage

The 512 KiB idstorage partition at 0x00040000 is **reconstructed**: its content
is personal to each console (it holds the per-unit keys) and is not in the PUP.
What the builder actually writes is described in §5.1, which matches the code
(`build_idstorage_image()` in `src/hw/emmc/emmc_image.cpp`) and the reference dump;
the earlier description of "256 leaf numbers at 0x00, leaves 1..255 registered"
was wrong and is not repeated here. This is a placeholder, not a real ID storage:
only leaf 0 (the mapping table) and leaf 192 (the SMI list) carry data, everything
else is `0xFF`.

---

## 6. Verification

`emmc_rebuild --verify` re-reads the finished image and checks:

* the master block magic, version, signature and every partition record;
* all four SLB2 copies: magic, version, table size, entry count, each entry's
  name / offset / size and the magic of each payload (MeP or SCE);
* both FAT16 volumes: BPB, FAT size, FSInfo, root directory, every cluster chain;
* every file against `Out/fs/os0` / `Out/fs/vs0`, byte for byte.

Observed on the 512 MiB image and on the full 3.55 GiB image:

```
verify OK
  master block : version 3, total 7446528 blocks (3.55 GiB), 12 partition records
  SLB2         : 7 entries: second_loader.enp@0x200+0x16C00(MeP), second_loader.enc@0x16E00+0x16C00(MeP),
                 secure_kernel.enp@0x2DA00+0x8200(MeP), secure_kernel.enc@0x35C00+0x8200(MeP),
                 kernel_boot_loader.self@0x3DE00+0x56B94(SCE), kprx_auth_sm.self@0x94A00+0x88F8(SCE),
                 prog_rvk.srvk@0x9D400+0x6C0(SCE) (4 copies checked)
  os0_0    mounted: 4096 B/cluster, 4087 clusters, 63 files, 4 dirs, label "NO NAME", 63/63 files verified
  vs0_0    mounted: 4096 B/cluster, 65467 clusters, 929 files, 230 dirs, label "NO NAME", 929/929 files verified
  files checked: 992, mismatches: 0
```

Independent cross-checks that were run:

* **7-Zip** reads both volumes out of the image (`7z t` on the extracted
  partition: os0 -> 63 files / 6,405,765 bytes / 4 folders, vs0 -> 929 files /
  84,375,093 bytes / 230 folders, CRC pass).
* The partition image bytes in the finished image are byte identical to
  `PUP_dec/os0.bin` / `vs0.bin`, with a zero filled tail.
* The eMMC boot partitions and the two `bls` copies are byte identical to each
  other (first 0xA000 bytes).
* End to end: `zeliboba.exe -q -ex "boot" -ex "runm 300000" -ex "boot" -ex "quit"`
  reads the container out of the image through the emulated card and stages it -

  ```
  [info ] machine    SLB2 from eMMC SLB2: 7 entries
  [info ] boot         SLB2 entry second_loader.enp        offset=0x200 size=93184
  [info ] boot         SLB2 entry second_loader.enc        offset=0x16E00 size=93184
  [info ] boot         SLB2 entry secure_kernel.enp        offset=0x2DA00 size=33280
  [info ] boot         SLB2 entry secure_kernel.enc        offset=0x35C00 size=33280
  [info ] boot         SLB2 entry kernel_boot_loader.self  offset=0x3DE00 size=355220
  [info ] boot         SLB2 entry kprx_auth_sm.self        offset=0x94A00 size=35064
  [info ] boot         SLB2 entry prog_rvk.srvk            offset=0x9D400 size=1728
  [info ] boot       ARM boot ROM staged second_loader.enc at 0x407C0000 (93184 bytes)
  ```

  The CMeP first loader then reports `first loader reported SUCCESS to the ARM
  mailbox` and hands control to `0x40000`, so the container is found, parsed,
  validated by the loader's own crypto chain and executed.

### What could not be verified

* **No real 1.04 eMMC dump exists**, so the reconstruction is validated against
  the PUP, the extracted tree and the machine's own boot path - not against a
  genuine 1.04 device image. The only eMMC dump available
  (`dumps/emmcdump.zip`, 24 MiB of a bricked, different console) was used for
  the master block / partition table / SLB2 **format** only.
* The **idstorage**, **sa0, tm0, vd0, ud0, pd0 and ur0** partitions have no
  source data; the first is a structural placeholder and the rest are erased.
* os0/tm0/vd0/ud0 are FAT16 and pd0/ur0 are exFAT on hardware; we have no
  content for any of them and no exFAT implementation, so no exFAT volume is
  produced.
* CID serial / manufacturing date and several EXT_CSD power and performance
  fields are datasheet defaults rather than measured values.
* Windows cannot mount the image without administrator rights
  (`Mount-DiskImage`), so mounting was done with 7-Zip and with the project's
  own FAT parser instead of the Windows filesystem driver.

### 5.1 Дополнение (раунд 21): таблица отображения и SMI-лист **[Ф]/[Р]**

Формат idstorage подтверждён вики (<https://wiki.henkaku.xyz/vita/IdStorage>, дал
пользователь) и кодом второй стадии:

* в начале раздела — **таблица отображения**: массив `u16` идентификаторов листьев;
  записи самой таблицы помечены `0xFFF5`, свободные — `0xFFFF`;
* листья по 512 байт; данные листа с *индексом* `i` лежат по `512*i` (индекс
  берётся из таблицы по идентификатору, а не равен ему);
* дамп по 0x40000 читается ровно так: 32 записи `0xFFF5`, затем идентификаторы
  `0x0000..0x007F` в слотах 32..159, `0xFFFF` в 160..191 и список
  `0x0080, 0x0100, 0x0102, 0x0103, 0x0110..0x0115` в слотах 192..201;
* **лист с идентификатором `0x80` — SMI** (Service/Manufacturing Information).
  Его структура: `"SMI\0"`, `u32 version = 1`, `u32 minfw_plaintext`, нули до
  0x7F (проверяются `second_loader`), затем payload (0x80..0xFF) и подпись
  (0x100..0x1FF). Payload зашифрован **дважды** (outer+inner), подпись — один
  раз, ключами, выводимыми из per-console keyslot `0x213`; область 0x00..0x7F
  не шифруется. Именно эту проверку выполняет вторая стадия: она ищет `0x0080`
  в таблице (0x4678C), читает слот 192 (блок eMMC 0x2C0) и расшифровывает
  payload двумя проходами (0x464C0/0x46632/0x4687C).

Сборщик образа (`build_idstorage_image`) пишет таблицу по дампу и синтезирует
SMI-лист: `"SMI\0"`, version 1, `u32 minfw_plaintext = 0` и нули до 0x7F; payload
(0x80..0xFF) — нули, подпись (0x100..0x1FF) — `0xFF`, потому что на железе они
зашифрованы ключом консоли, которого в дампах нет.
