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

The right hand is controlled from the Orin over Modbus TCP (default `192.168.123.211:6000`, override with `INSPIRE_HAND_IP`). It has 6 actuated DoF (pinky, ring, middle, index, thumb bend, thumb rotation) driving 12 underactuated joints.

- DimOS module: `dimos/robot/unitree/g1/effectors/inspire_hand/` (`InspireHandModule`), blueprint `unitree-g1-inspire-hand`.
- Streams: `hand_command` (6-D `JointState`, normalized 0 to 1, 1 = open) and `hand_state`. RPC: `open_hand`, `close_hand`, `set_normalized`.
- Start on Orin: `./run_g1_inspire_hand.sh` (also `status`, `logs`, `stop`, `--demo` for a short Modbus-only wiggle). The hand opens on Ctrl-C.
- Gesture demo (needs the module running): `./run_g1_inspire_hand_gestures.sh` (finger-by-finger bends, yeah, thumbs-up; `REPEAT`, `HAND_CLOSE`, `HAND_FIST` at the top).
- Standalone Modbus test without DimOS: `./run_g1_inspire_right_hand_test.sh`.

The scripts use `DIMOS_TRANSPORT=lcm`. Thumb parameters for the yeah gesture still need physical tuning.
