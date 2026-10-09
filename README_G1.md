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
