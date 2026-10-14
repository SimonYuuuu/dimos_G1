# dimos_G1

G1 navigation and bilingual onboard voice control, based on DimensionalOS DimOS.

This repository starts from the local code snapshot at `d5e74fe5d99237b7c72bf5aa2e9c1a550a1cbfdc`. The upstream Apache-2.0 license and source notices are retained. Original upstream: https://github.com/dimensionalOS/dimos

## Scope

Source code, tests, dependency manifests, configuration templates, and G1 launch/deployment scripts are versioned. Local maps, recordings, environments, logs, and upstream Git LFS datasets/media/simulation assets are not included. Dataset-dependent tests, simulations, and documentation images may require separately obtaining the upstream assets. Existing local assets are preserved.

## Deployment

The development checkout is `/home/cy/G1/dimos`; deployment on Orin is `~/dimos`. Hardware launch scripts assume the Orin environment and its installed dependencies.

- Start voice control on Orin: `./run_g1_dimos_voice_control.sh`
- Monitor voice: `./run_g1_voice_monitor.sh`
- Stop: `./run_g1_dimos_voice_control.sh stop`

Keep local environment variables and credentials outside Git. Existing navigation and voice changes still require physical validation.

## Inspire RH56DFTP right hand

The right hand is controlled from the Orin over Modbus TCP (default `192.168.123.211:6000`). It has 6 actuated DoF (pinky, ring, middle, index, thumb bend, thumb rotation) driving 12 underactuated joints.

- DimOS module: `dimos/robot/unitree/g1/effectors/inspire_hand/` (`InspireHandModule`), blueprint `unitree-g1-inspire-hand`.
- Address: `INSPIRE_HAND_IP` / `INSPIRE_HAND_PORT` env vars are read by the module config (`ip`, `port`), the Modbus client and all scripts; an explicit config value wins over the env var.
- Streams: `hand_command` (6-D `JointState`, normalized 0 to 1, 1 = open, full range), `hand_state`, and `hand_status` (`connected` / `disconnected: <reason>`).
- Reconnect: if the link drops the module logs a warning, publishes `hand_status`, and reconnects with exponential backoff (`reconnect_initial_s` 0.5 s up to `reconnect_max_s` 5 s). Modbus responses are checked for transaction id, unit id and function code; a mismatch or a closed peer drops the socket and triggers reconnect.
- Safety: agent skills (`set_hand_pose`, `close_hand`, `open_hand`) cannot close the fingers/thumb bend below `skill_min_opening` (default 0.3; thumb rotation is not limited). Full-range control is RPC-only: `set_normalized`, `set_raw`, plus the `hand_command` stream. Speed/force are capped by `max_speed` / `max_force` (default 500; `default_speed`/`default_force` 200), also via RPC `set_speed_force`.
- Start on Orin: `./run_g1_inspire_hand.sh` (also `status`, `logs`, `stop`, `--demo` for a short Modbus-only wiggle). The hand opens on Ctrl-C.
- Gesture demo: `./run_g1_inspire_hand_gestures.sh` runs `python -m dimos.robot.unitree.g1.effectors.inspire_hand.gestures` (finger-by-finger bends, yeah, thumbs-up; `REPEAT`, `HAND_CLOSE`, `HAND_FIST` at the top). It reuses a running `unitree-g1-inspire-hand` instance, otherwise starts one and stops only that instance on exit; it exits with an error if a different DimOS blueprint is running or if the hand does not connect within `START_TIMEOUT` (30 s). The hand is always reopened on exit.
- Standalone Modbus test without DimOS: `./run_g1_inspire_right_hand_test.sh` (defaults: speed/force 200, `HAND_CLOSE` 850; `--remote` runs it on the Orin over SSH).
- Unit tests (no hardware): `uv run pytest dimos/robot/unitree/g1/effectors/inspire_hand`.

Thumb parameters for the yeah gesture still need physical tuning.

## Orin 代码同步（Orin 无外网）

本机（host）是中转站：GitHub ⇄ 本机 `/home/cy/G1/dimos` ⇄ Orin。

- Orin 上有一个 bare 仓库 `~/git/dimos_G1.git`，由本机通过局域网推送。
- Orin 的工作目录 `~/dimos` 是从这个 bare 仓库 clone 的，`origin` 指向 `/home/unitree/git/dimos_G1.git`。
  （不直接指向本机：Orin 访问本机需要本机开 SSH 服务和密钥，用 bare 仓库更简单可靠。）
- 本机 remote：`origin` = GitHub，`orin` = `unitree@192.168.123.164:git/dimos_G1.git`。

### 本机

```bash
cd /home/cy/G1/dimos
git fetch origin                 # 从 GitHub 拉
git checkout <branch> && git pull
git push origin <branch>         # 推到 GitHub
git push orin <branch>           # 推到 Orin 的 bare 仓库（不要用 -f）
```

首次配置：`git remote add orin unitree@192.168.123.164:git/dimos_G1.git`

### Orin

```bash
cd ~/dimos
git status                       # 切分支前先 commit 或 stash，否则 git 会拒绝覆盖
git fetch origin
git checkout <branch>            # 或 git pull 更新当前分支
```

在 Orin 上改了代码：`git commit` 后 `git push origin <branch>`（推到 bare 仓库），
再在本机 `git fetch orin && git merge orin/<branch>`，然后推 GitHub。

### 注意

- `.venv`（uv）、`target/`、Rust/C++ 编译产物、`data/g1_urdf`、`data/groot` 不在 git 里，是从旧目录复制过来的；
  venv 里的脚本写死了 `/home/unitree/dimos` 路径，所以目录不要改名。运行用 `uv run --no-sync ...`，Orin 无外网，不要 `uv sync`。
- 地图 `g1_map.pc2.lcm`、`dimos/map/g1_map.pc2.lcm`、日志、pid、`.env`/本地配置都被 `.gitignore` 忽略，不要提交。
- 切换前的旧目录（跟 upstream 历史）完整保留在 Orin 的 `~/dimos_upstream_bak_20261014`（含 `recordings/`），
  本机另有备份 `/home/cy/G1/orin_backup_20261014_1417/`。不要删除。
- 同一台机器并行开发多个分支时，用 `git worktree add ../wt/<name> <branch>`。
