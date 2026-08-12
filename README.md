# v8patch

Community patches and quality of life changes for 2007 Roblox clients. (0.3.676.0 only for now!)

## Settings

`v8patch.ini` sits next to the dll. A missing file, section or key falls back to
the following defaults:

| Section     | Key       | Default | Meaning                                                                   |
| ----------- | --------- | ------- | ------------------------------------------------------------------------- |
| `framerate` | `enabled` | `1`     | Apply the framerate patch.                                                |
| `framerate` | `fps`     | `60`    | Frames a second. `0` removes the cap.                                     |
| `physics`   | `enabled` | `1`     | Replace the engine solver with Box3D.                                     |
| `harness`   | `enabled` | `0`     | Log script output and let a `-script` file drive a place, for benchmarks. |

## Building

You will need a 32-bit MSVC toolchain to build this project.

```
cmake --preset x86
cmake --build --preset x86
```
