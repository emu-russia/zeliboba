# Prepared cold native SDL comparison — 2026-10-01

Execution is held until the parent confirms the next native producer/SFB result and authorizes the capture. No new SDL process, clone or screenshot has been created by preparing this procedure.

`goal-native-cold-stack-macos-ui-capture.py` prints its plan by default. On GO, execute it with `--execute` against the already integrated binary. It clones `goal-emmc.img` using APFS `cp -c`, removes every inherited `ZLB_*` variable, uses absolute paths and `--no-rebuild`, and runs:

```
zeliboba_ui --emmc <fresh clone> --no-rebuild --run 0 \
  -ex 'runm 1000000' -ex devices -ex emmc -ex uart \
  --screenshot <fresh bmp> --screenshot-tab panel --screenshot-frames 2
```

The script refuses to overwrite earlier `.img/.bmp/.png/.log/.json` artifacts. It does not signal or change existing live UI PID `20100`; it records nondestructive process-state checks before and after the new UI's normal exit. It records the source image's size/mtime before and after, SDL stdout/stderr, command, fresh PID and exit status. Conversion uses `sips`, then the resulting PNG must be inspected with `view_image`.

Compare the actual panel region with `goal-native-checksum-macos-ui.png`, the prior 611 capture, and its SDL log. That earlier capture had a Metal renderer, working 48 kHz stereo audio stream and exit zero, but showed a host `no framebuffer yet` placeholder. The placeholder explanation text has since been corrected, so a text-only screenshot difference cannot establish guest progress. Only observed guest pixels plus the corresponding native framebuffer state may establish a logo. No display asset is loaded or inserted by this procedure.

Report the fresh renderer/audio/exit result, source-preservation check and actual panel appearance. If the panel is blank or still displays the host placeholder, say so. Do not infer an os0 boot or logo from successful SDL initialization, head enable, DSI frame counts or the screenshot file existing.
