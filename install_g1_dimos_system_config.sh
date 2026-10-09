#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

install -d -m 0755 /etc/sysctl.d /etc/systemd/system

tmp_sysctl="$(mktemp)"
tmp_service="$(mktemp)"
trap 'rm -f "${tmp_sysctl}" "${tmp_service}"' EXIT

cat >"${tmp_sysctl}" <<'EOF'
# DimOS/LCM receive buffers. Applied by systemd-sysctl on every boot.
net.core.rmem_max = 67108864
net.core.rmem_default = 67108864
EOF

cat >"${tmp_service}" <<'EOF'
[Unit]
Description=Configure loopback multicast routing for DimOS LCM
After=network.target
Before=multi-user.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link set dev lo multicast on
ExecStart=/sbin/ip route replace 224.0.0.0/4 dev lo
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

install -m 0644 "${tmp_sysctl}" /etc/sysctl.d/99-dimos-lcm.conf
install -m 0644 "${tmp_service}" /etc/systemd/system/dimos-lcm-network.service

sysctl --system >/dev/null
systemctl daemon-reload
systemctl enable --now dimos-lcm-network.service

echo "DimOS LCM system configuration installed."
ip -details link show lo | head -n 2
ip route show 224.0.0.0/4
sysctl net.core.rmem_max net.core.rmem_default
