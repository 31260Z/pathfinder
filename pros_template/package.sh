#!/usr/bin/env bash
# Build a `pros conduct`-installable Pathfinder template .zip.
#
#   bash pros_template/package.sh
#
# Steps:
#   1. Mirror <repo>/include/pathfinder/ into <repo>/pros_template/include/
#      so the headers ship inside the template archive.
#   2. (No-op for v0.1 — the library is header-only. v0.2 will compile a
#      static lib here and drop it into firmware/libpathfinder.a.)
#   3. Zip the staging dir into pathfinder@<version>.zip in the repo root.
#   4. Print the install commands for the user.

set -euo pipefail

# Resolve the repo root from the script's location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TEMPLATE_DIR="$SCRIPT_DIR"

# Pull the version out of template.pros (the source of truth).
VERSION="$(grep -E '"version"' "$TEMPLATE_DIR/template.pros" \
            | head -1 | sed -E 's/.*"version":[[:space:]]*"([^"]+)".*/\1/')"
if [[ -z "$VERSION" ]]; then
    echo "package.sh: failed to parse version from template.pros" >&2
    exit 1
fi
ARCHIVE_NAME="pathfinder@${VERSION}.zip"
ARCHIVE_PATH="$REPO_ROOT/$ARCHIVE_NAME"

echo "[package.sh] Pathfinder ${VERSION}"

# 1. Mirror headers.
echo "[package.sh] Copying headers from include/ → pros_template/include/"
rm -rf "$TEMPLATE_DIR/include"
mkdir -p "$TEMPLATE_DIR/include"
cp -R "$REPO_ROOT/include/pathfinder" "$TEMPLATE_DIR/include/pathfinder"

# 2. (Future) compile static lib. v0.1 is header-only.
mkdir -p "$TEMPLATE_DIR/firmware"
echo "[package.sh] No static lib to build (header-only library); firmware/ kept empty."

# 3. Zip the contents (NOT the parent dir).
echo "[package.sh] Building $ARCHIVE_NAME"
rm -f "$ARCHIVE_PATH"
(
    cd "$TEMPLATE_DIR"
    zip -rq "$ARCHIVE_PATH" \
        template.pros \
        include \
        firmware \
        src \
        README.md
)
echo "[package.sh] Created $ARCHIVE_PATH"

# 4. Print install commands.
cat <<EOF

Install in your PROS project:

    cd /path/to/your/pros_project
    pros conduct fetch $ARCHIVE_PATH
    pros conduct apply pathfinder@${VERSION}
    pros mu        # build + upload

EOF
