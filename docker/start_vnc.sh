#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
export XVFB_GEOM="${XVFB_GEOM:-1920x1080x24}"

Xvfb "${DISPLAY}" -screen 0 "${XVFB_GEOM}" >/tmp/xvfb.log 2>&1 &
fluxbox >/tmp/fluxbox.log 2>&1 &
x11vnc -display "${DISPLAY}" -forever -shared -nopw -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc/ 6080 localhost:5900 >/tmp/novnc.log 2>&1 &

sleep 1

# Visible marker in VNC so a black screen means Gazebo/gui issue, not noVNC transport.
xsetroot -display "${DISPLAY}" -solid "#1f2430" || true
xterm -display "${DISPLAY}" -geometry 120x20+20+20 -fa Monospace -fs 10 -e "bash -lc 'echo VNC desktop ready on ${DISPLAY}; echo Waiting for Gazebo...; tail -f /tmp/xvfb.log /tmp/fluxbox.log /tmp/x11vnc.log /tmp/novnc.log'" >/tmp/xterm.log 2>&1 &
