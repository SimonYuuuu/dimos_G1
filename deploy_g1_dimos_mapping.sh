#!/usr/bin/env bash
# Copy only the passive mapping additions to the Orin.  Deliberately does not
# pull, reset, or otherwise modify the remote working tree's existing changes.
set -euo pipefail

remote="${G1_SSH_HOST:-g1-dimos}"
remote_root="${G1_DIMOS_DIR:-/home/unitree/dimos}"

ssh -o BatchMode=yes "$remote" "test -d '$remote_root/.git'"

rsync -av \
  dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_map_record.py \
  dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_relocalization.py \
  "$remote:$remote_root/dimos/robot/unitree/g1/blueprints/navigation/"
rsync -av \
  dimos/mapping/relocalization/module.py \
  "$remote:$remote_root/dimos/mapping/relocalization/"
rsync -av \
  dimos/robot/all_blueprints.py \
  "$remote:$remote_root/dimos/robot/"
rsync -av \
  run_g1_dimos_mapping.sh \
  run_g1_dimos_relocalization.sh \
  "$remote:$remote_root/"
rsync -av g1_map.pc2.lcm "$remote:$remote_root/"
ssh -o BatchMode=yes "$remote" "chmod +x '$remote_root/run_g1_dimos_mapping.sh' '$remote_root/run_g1_dimos_relocalization.sh'"

echo "Deployed passive mapping recorder to $remote:$remote_root"
