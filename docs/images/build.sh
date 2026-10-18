#!/bin/bash
# Usage: ./build.sh [--force] [name.asy ...]
# https://asymptote.sourceforge.io/doc/colors.png

asy="/c/Program Files/Asymptote/asy.exe"
if [ ! -x "$asy" ]; then
  echo "missing $asy"
  exit 1
fi

cd "$(dirname "$0")" || exit 1

force=0
sources=()
for arg in "$@"; do
  case "$arg" in
    --force) force=1 ;;
    *) sources+=("$arg") ;;
  esac
done

if [ ${#sources[@]} -eq 0 ]; then
  sources=(*.asy)
fi

# asy can exit 0 after a failed TeX run, so judge success by a fresh svg
marker=$(mktemp)
trap 'rm -f "$marker"' EXIT

status=0
failed=0
for src in "${sources[@]}"; do
  name=$(basename "$src" .asy)
  svg="$name.svg"

  # Never clobber a hand drawn svg of the same name, delete it first to replace it
  if [ -f "$svg" ] && ! head -c 300 "$svg" | grep -q dvisvgm; then
    echo "skip   $src: $svg was not made by asy"
    status=1
    continue
  fi

  if [ $force -eq 0 ] && [ "$svg" -nt "$src" ]; then
    echo "fresh  $svg"
    continue
  fi

  touch "$marker"
  "$asy" -noV -o "$name" "$src"
  if [ "$svg" -nt "$marker" ]; then
    echo "built  $svg"
  else
    echo "failed $src"
    status=1
    failed=1
  fi
done

# Leftovers from asy's TeX pipe. Keep the log after a failure, it has the TeX error.
rm -f texput.aux
if [ $failed -eq 0 ]; then
  rm -f texput.log
fi
exit $status
