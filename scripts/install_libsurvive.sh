#!/usr/bin/env bash
# Build & install libsurvive + the pysurvive python bindings, and install the
# udev rules this package needs.  Steam / SteamVR is NOT required.
#
# Usage:
#   rosrun vive_libsurvive_ros install_libsurvive.sh           # build in ~/libsurvive
#   SRC_DIR=/opt/src rosrun vive_libsurvive_ros install_libsurvive.sh
set -euo pipefail

SRC_DIR="${SRC_DIR:-$HOME/libsurvive}"
REPO="https://github.com/collabora/libsurvive.git"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> Installing build dependencies (sudo apt)..."
sudo apt-get update
sudo apt-get install -y build-essential cmake zlib1g-dev libx11-dev \
    libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev \
    libatlas-base-dev python3-pip python3-dev

echo "==> Cloning / updating libsurvive in $SRC_DIR ..."
if [ -d "$SRC_DIR/.git" ]; then
    git -C "$SRC_DIR" pull --ff-only
    git -C "$SRC_DIR" submodule update --init --recursive
else
    git clone --recursive "$REPO" "$SRC_DIR"
fi

echo "==> Building libsurvive (C library + tools)..."
cd "$SRC_DIR"
make -j"$(nproc)"

echo "==> Installing libsurvive system-wide..."
sudo make install
sudo ldconfig

echo "==> Installing pysurvive python bindings..."
# Build the bindings against the freshly built tree.
if ! python3 -c "import pysurvive" 2>/dev/null; then
    pip3 install --user . || pip3 install --user pysurvive
fi
python3 -c "import pysurvive; print('pysurvive OK:', pysurvive.__file__)"

echo "==> Installing udev rules..."
sudo cp "$PKG_DIR/udev/83-vive-libsurvive.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

cat <<'EOF'

==> Done.
Next steps:
  1) Replug the dongles / USB tracker (or reboot) so udev permissions apply.
  2) Power on both base stations.
  3) Verify libsurvive sees everything (prints serials of trackers + lighthouses):
         survive-cli
  4) Do a one-time room calibration (hold a tracker still, visible to both LHs):
         survive-cli   # let it converge, it writes ~/.config/libsurvive/config.json
  5) Launch the ROS node:
         roslaunch vive_libsurvive_ros libsurvive_tracking.launch
EOF
