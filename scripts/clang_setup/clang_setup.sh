#!/usr/bin/env bash

set -euo pipefail

script_dir="$( (builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd) )"
repo_root="$( ( builtin cd "$script_dir" && git rev-parse --show-toplevel) )"

cat <<EOF
Setup script.
  This script will:
    - install clang and clang-format
    - setup the clang-format pre-commit hook
  \`sudo\` will be invoked by several of these stages.
EOF

if (( $# == 0 )); then
  echo
  read -p "Press 'Y' to continue, or any other key to cancel: [y/N]: "
  if [ "$REPLY" != "y" ] && [ "$REPLY" != "Y" ]; then
    echo "Exiting."
    exit 1
  fi
fi

# Simple single retry function that sleeps 5 seconds before trying again.
function retry {
  if ! $@; then
    echo Command failed: $@
    echo Retrying in 5 seconds...

    sleep 5

    $@
  fi
}


echo -------------------------------------------------------
echo Installing packages...

sudo apt-get update && \
sudo apt-get install --assume-yes --no-install-recommends \
  clang \
  clang-format


echo -------------------------------------------------------
echo Setting up clang...

sudo cp "$script_dir/clang/git-clang-format" /usr/local/bin

if [ -d "$repo_root/.git" ]; then
  cp "$script_dir/pre-commit" "$repo_root/.git/hooks/"
else
  # Check if .git is a file. It means enav is installed as a submodule.
  # In that case we try to install pre-commit hook into the supermodule directory.
  if [ -f "$repo_root/.git" ]; then
    dot_git_content=$(<".git")
    echo dot_git_content
    git_module_path=${dot_git_content#"gitdir: "}
    echo git_module_path
    if [ -d $git_module_path ]; then
      echo Installing clang format to $git_module_path/hooks
      cp "$script_dir/pre-commit" $git_module_path/hooks
    else
      echo Failed to install clang format hooks because supermodule hooks directory not found: $git_module_path/hooks
    fi
  fi
fi
