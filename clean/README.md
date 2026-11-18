# Clean workspace: A / H / S

This directory is the clean working area. The original `RePart/` tree and `runs/`
remain untouched as legacy/experiment archives.

## Layout

- `baseline/` — A baseline. Source in `baseline/RePart/`, no experiment macros.
- `H/` — community-guided search variant. Shares `baseline/RePart/` source and
  adds `REPART_COMMUNITY_SEARCH` plus a separately compiled `detector.o`.
- `S/` — global assignment scoring variant. Shares `baseline/RePart/` source and
  adds `REPART_GLOBAL_ASSIGNMENT`; board order/schedule and theta are runtime
  switches.
- `third_party/boost_1_86_0/` — vendored boost used by all builds.
- `build_all.sh` — builds all three binaries into their `build/` directories.

The variants share source intentionally: A/H/S are compile-time/runtime
configurations of one implementation. Each variant has its own build and run
entrypoint so they can be used independently without editing the others.

## Build

Builds require a Linux/WSL g++ (the project's existing toolchain); Git Bash on
Windows does not provide `g++`. On Windows, run the script inside WSL:

```bash
wsl -d Ubuntu-24.04 -u sjy -- bash -c \
  'cd /mnt/c/Users/86139/RePart-main/RePart-main && bash clean/build_all.sh'
```

Inside WSL/Linux, simply:

```bash
bash clean/build_all.sh
```

Outputs:

- `clean/baseline/build/partitioner_A`
- `clean/H/build/partitioner_H`
- `clean/S/build/partitioner_S`

## Run

```bash
# A
bash clean/baseline/run.sh testcase/case01 /tmp/case01_A.out

# H (delay 0 by default)
bash clean/H/run.sh testcase/case01 /tmp/case01_H.out

# S, default theta and first-complete stop
bash clean/S/run.sh testcase/case01 /tmp/case01_S.out

# S with resources removed and full search
THETA=0 FIRST_ONLY=0 bash clean/S/run.sh testcase/case01 /tmp/case01_S_theta0_search.out
```

All run scripts apply `ulimit -v` 6 GiB by default; override with `MEM_KB=...`.

## Notes

- `runs/` is the experiment archive; reports and data there are intentionally
  kept in place so relative paths and evidence links stay valid.
- `docs/` is the documentation index; per-run `REPORT.md` files stay with their
  run data.
- Do not edit `clean/baseline/RePart` separately for a variant; put variant
  switches/scripts in `clean/H` or `clean/S`.
