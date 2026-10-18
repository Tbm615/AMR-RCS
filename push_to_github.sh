#!/bin/bash
# One-shot: create github.com/<you>/AMR-RCS as a public repo and push this
# local git history to it. Run this from the RCS folder.
#
# Prerequisites: run `gh auth login` once in the same shell (device code
# or token flow); then run this script.
set -euo pipefail

cd "$(dirname "$0")"
GH="./.tools/gh_2.60.1_linux_amd64/bin/gh"
[ -x "$GH" ] || GH=$(command -v gh)

if ! $GH auth status >/dev/null 2>&1; then
    echo "You are not logged in to GitHub yet. Run first:"
    echo "   $GH auth login"
    exit 1
fi

# Create the repo if it doesn't exist yet, and push.
if $GH repo view AMR-RCS >/dev/null 2>&1; then
    echo "Repo already exists; pushing new commits."
    git push -u origin main
else
    echo "Creating github.com/<you>/AMR-RCS and pushing..."
    # Overwrite the pre-set origin with whatever gh picks for your account.
    git remote remove origin 2>/dev/null || true
    $GH repo create AMR-RCS --public --source=. --remote=origin --push \
        --description "Standalone desktop operator console for Hinson-class AMRs. Windows + Linux, no ROS 2 needed on the operator PC."
fi

echo
echo "Done. Open the repo:"
$GH repo view --web AMR-RCS
