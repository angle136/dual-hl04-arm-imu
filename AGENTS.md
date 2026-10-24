# Agent Notes

## Build

- This repository has no root CMake project; configure from a board source directory (`board/dji_c`, `board/damiao_h7`, or `board/f103_c8`) and use the ARM bare-metal GCC toolchain.
- Required host tools are `cmake`, `ninja`, and `arm-none-eabi-gcc`; `ccache` is used automatically when installed. `cppcheck` is required only when explicitly configured with `-DMAS_REQUIRE_CPPCHECK=ON`.
- A focused build is:
  `cmake -S board/dji_c -B build/dji_c/Debug --preset Debug -DROBOT=arm -DBOARD=single`
  followed by `cmake --build build/dji_c/Debug -j "$(nproc)"`.
- `ROBOT` and `BOARD` are CMake cache variables. Existing build directories retain old values, so override them with `-D...` or use a new/remove the build directory when switching configurations.
- Configuration generates `robot_def.h` and `module_config.h` under the build directory; do not edit generated headers. The firmware ELF is `build/<board>/<config>/base.elf`.
- The root build has no host unit-test or `ctest` workflow. The meaningful local checks are a target-board build and `cmake --build <build-dir> --target cppcheck-log` (when `cppcheck` is installed); the report is under `<build-dir>/cppcheck/cppcheck.log`.

## Layout

- `cmake/board_common.cmake` is the shared build entrypoint. Board `CMakeLists.txt` files select the ThreadX architecture, while `apps/config.cmake` selects robot/board-role settings and `apps/<robot>/robot.cmake` selects modules and hardware parameters.
- Application configurations are discovered by CI from `apps/*/robot.cmake` and each matching `*_board` directory. Keep module source registration in `modules/CMakeLists.txt` and module defaults in `modules/module_config.cmake`.
- `build/` is ignored generated output. IDE build tasks also copy the active `compile_commands.json` to `build/` and the ELF to `build/<board>.elf` for clangd/debugging.

## Change Boundaries

- When adding a module, change only the files required for that module and its explicit registration/configuration; do not modify unrelated modules, BSP code, application code, or formatting incidentally.
- Keep layer ownership intact: an application/module change must not alter a lower layer merely to accommodate an upper-layer design. Change a lower layer only when it is necessary for the required behavior, and verify its other callers.
- Keep the patch and affected surface as small as possible; do not add speculative abstractions, compatibility code, or scaffolding.

## Arm Engineering Operations

- Keep every change as small and focused as possible. For an application-layer arm issue, prefer changing only `apps/arm`.
- Do not make incidental changes outside the application layer, including `modules` or `board/bsp`. If an out-of-APP change is necessary, explain the reason and affected behavior before editing it.
- Preserve existing worktree changes. Do not overwrite or revert changes whose origin is unknown.
- After modifying C or C++ files, run the repository's `clang-format` and `git diff --check`.
- After any code change, build the `dji_c / arm / single / Debug` target.
- Do not flash firmware unless the user explicitly requests it.
- Do not commit or push unless the user explicitly requests it.
- Before committing, inspect the actual diff, current branch, and remotes, and exclude unrelated files from the commit.
- When the user requests a read-only diagnostic phase, do not modify files, build, flash, commit, or push.

## Checks And Workflow

- Format changed C/C++ files with the repository `.clang-format` (`clang-format -i <files>`). Cppcheck/clang-tidy settings are in `.clang-tidy`; CI's `cppcheck-log` covers only `board/bsp`, `modules`, `apps`, and `utils`.
- CI builds every discovered robot/board-role configuration for `damiao_h7` and `dji_c`, then runs cppcheck for merge requests targeting `dev`. It does not validate `f103_c8`.
- `main` accepts merges only from this repository's `dev` branch. Pushing to `dev` triggers the automated merge into `dev-systemview`.
- `.gitattributes` marks `threadx/**` and `utils/**` as `merge=ours` for the `dev` to `dev-systemview` merge. Before doing that merge locally, register the driver with `git config merge.ours.driver true`.
- Flashing is hardware-dependent and supports only `damiao_h7` and `dji_c`; use `.vscode/flash_interactive.sh <board> <probe>` after building, with probe `stlink`, `daplink`, or `jlink`.

## Arm Transmitted-Angle Safety Invariant

The mapping from motor feedback to the J1-J7 floats sent by `board_link` is a fixed external contract. Do not change it unless the user explicitly requests a protocol-coordinate change.

### Sources And Forbidden Substitutions

- J1, J2, J3, J5, J6, and J7 use the DM driver's decoded `single_round_angle`.
- J4 uses the GM6020 driver's continuous `total_angle`, including `total_round`.
- Do not change J4 back to `single_round_angle`.
- Send `joint_angle_rad`, not `model_angle_rad`. In particular, J3's gravity-model `+pi` must never enter its transmitted value.
- Do not add angle wrapping, normalization, hidden sign changes, or extra `+pi`/`-pi` operations to the transmit path.

### Required Equations

Let `r1`, `r2`, `r3`, `r5`, `r6`, and `r7` be the corresponding DM `single_round_angle` values, and let `t4` be J4 `total_angle`. The transmitted values must remain:

```text
J1_send = r1 + 1.836
J2_send = r2 + 0.952
J3_send = r3 - 3.148
J4_send = t4 - 1.01242732
J5_send = 0.040 - r5
J6_send = r6 - 1.04290104
J7_send = r7 - 0.066
```

J2 contains two deliberate sign changes:

```text
q2      = -(r2 + 0.952)
J2_send = -q2
```

J6 contains a transmit-only offset:

```text
q6      = r6 - 1.086
J6_send = q6 + 0.04309896
```

### Raw Feedback Equations

For the current DM position range, a 16-bit raw position `U` decodes as:

```text
r = -12.5 + U * 25 / 65535
```

The current GM6020 implementation computes:

```text
J4_single = ECD * 0.000767
t4        = total_round * 2*pi + J4_single
J4_send   = t4 - 1.01242732
```

J4 verification must therefore include both `ECD` and `total_round`; ECD alone cannot determine the continuous transmitted angle.

### Mandatory Check Vectors

For decoded driver angles:

```text
r1=-1.0, r2=0.0, r3=4.0, t4=2.0, r5=1.0, r6=2.0, r7=-1.0
```

the transmitted result must be:

```text
J1= 0.83600000
J2= 0.95200000
J3= 0.85200000
J4= 0.98757268
J5=-0.96000000
J6= 0.95709896
J7=-1.06600000
```

For raw feedback `U=32768` on every DM motor, and J4 `ECD=1320`, `total_round=0`, the transmitted result must be approximately:

```text
J1= 1.83619074
J2= 0.95219074
J3=-3.14780926
J4= 0.00001268
J5= 0.03980926
J6=-1.04271030
J7=-0.06580926
```

Every value must satisfy `abs(actual - expected) <= 1e-5 rad`.

### Required Review Procedure

Any change touching joint zero values, joint directions, angle decoding, `arm_motor_joint_angle()`, `arm_motor_model_angle()`, single/total-angle selection, transmit offsets, J2's transmit sign, board-link field order/type, or DM/DJI position decoding must:

1. State the seven expanded equations before the change.
2. State the seven expanded equations after the change.
3. Calculate both mandatory check vectors before and after the change.
4. Report the per-joint difference from the expected value.
5. Stop and identify the changed joint if any result exceeds `1e-5 rad`.

Do not update these expected equations or vectors in the same change merely to make a behavior change pass. They may be updated only when the user explicitly authorizes a change to the transmitted-coordinate contract.

## Ponytail

- First ask whether the change is needed; otherwise reuse existing code, then prefer standard-library/native/dependency facilities before adding new code.
- Prefer deletion and the smallest working diff. Do not add an abstraction with one implementation, a factory for one product, or configuration for an unchanging value.
- Understand the real flow before simplifying. For bug fixes, inspect all callers and fix the shared root cause rather than patching only the reported path.
- Do not simplify away validation, error handling, security, hardware calibration, or the smallest meaningful verification check.
- Mark deliberate known-ceiling shortcuts with a `ponytail:` comment that names the limitation and upgrade path.

`CMSIS-DSP/AGENTS.md` applies to work inside `CMSIS-DSP`.
