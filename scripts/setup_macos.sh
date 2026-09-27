#!/usr/bin/env bash
# One-time setup on macOS: builds Syphon.framework into third_party/ so the app can receive Syphon.
# Requires Xcode (full Xcode, not just the command line tools, is needed for xcodebuild).
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v cargo >/dev/null; then
  echo "Rust is not installed. Install it from https://rustup.rs and re-run." >&2
  exit 1
fi
if ! xcodebuild -version >/dev/null 2>&1; then
  echo "xcodebuild not available. Install Xcode from the App Store, then run:" >&2
  echo "  sudo xcode-select -s /Applications/Xcode.app/Contents/Developer" >&2
  exit 1
fi

mkdir -p third_party
if [ ! -d third_party/Syphon-Framework ]; then
  git clone --depth 1 https://github.com/Syphon/Syphon-Framework.git third_party/Syphon-Framework
fi

pushd third_party/Syphon-Framework >/dev/null
xcodebuild -project Syphon.xcodeproj -target Syphon -configuration Release \
  ARCHS="$(uname -m)" ONLY_ACTIVE_ARCH=YES CODE_SIGNING_ALLOWED=NO \
  SYMROOT="$PWD/build" build | tail -n 5
popd >/dev/null

FW=$(find third_party/Syphon-Framework/build -name Syphon.framework -type d -maxdepth 3 | head -n 1)
if [ -z "$FW" ]; then
  echo "Build finished but Syphon.framework was not found under third_party/Syphon-Framework/build" >&2
  exit 1
fi
rm -rf third_party/Syphon.framework
cp -R "$FW" third_party/Syphon.framework

# Let the binary find it via rpath, then re-sign (Apple Silicon refuses unsigned modified dylibs).
install_name_tool -id @rpath/Syphon.framework/Versions/A/Syphon third_party/Syphon.framework/Versions/A/Syphon
codesign --force --sign - third_party/Syphon.framework

echo
echo "Syphon.framework ready in third_party/."
if [ ! -e "/Library/NDI SDK for Apple/lib/macOS/libndi.dylib" ] && [ ! -e /usr/local/lib/libndi.dylib ]; then
  echo "NOTE: NDI runtime not found. For NDI input install the NDI SDK for Apple from https://ndi.video/for-developers/ndi-sdk/"
fi
echo "Now run:  cargo run --release"
