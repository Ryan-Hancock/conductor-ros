package gen

import (
	"fmt"
	"strings"
)

// InstallScript renders the bundle's install.sh. It is generated rather than
// embedded because everything it needs — app name, version, unit list,
// defaults — comes from the graph, and a script with the values already in it
// is one a robot operator can read and run by hand when there is no network
// to deploy over.
//
// The script is the only thing that touches the target's filesystem, so the
// same steps run whether `conductor deploy` invoked it over ssh or someone
// unpacked the tarball on the robot.
func InstallScript(d Deployment, units []string, keep int) string {
	if keep <= 0 {
		keep = 5
	}
	var b strings.Builder
	fmt.Fprintf(&b, `#!/usr/bin/env bash
# %s
#
# Installs the %s release %s.
#
#   ./install.sh                 install this release and restart the app
#   ./install.sh --no-restart    install, leave the running version alone
#   ./install.sh --no-systemd    copy files only (no units, no restart)
#   ./install.sh --rollback      switch back to the previously current release
#
set -euo pipefail

APP=%s
VERSION=%s
PREFIX=%s
SCOPE=%s
KEEP=%d
TARGET=%s
UNITS=(%s)
`,
		header,
		d.App, d.Version,
		shellQuote(d.App),
		shellQuote(d.Version),
		shellQuote(d.Prefix),
		shellQuote(d.Scope),
		keep,
		shellQuote(d.TargetName()),
		shellQuoteAll(units),
	)
	b.WriteString(installBody)
	return b.String()
}

// installBody is everything below the generated configuration block. It is a
// raw literal so the shell's own $ and % survive unedited.
const installBody = `
restart=1
systemd=1
rollback=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix) PREFIX="$2"; shift 2;;
    --scope) SCOPE="$2"; shift 2;;
    --keep) KEEP="$2"; shift 2;;
    --no-restart) restart=0; shift;;
    --no-systemd) systemd=0; shift;;
    --rollback) rollback=1; shift;;
    -h|--help) sed -n '3,10p' "$0"; exit 0;;
    *) echo "install.sh: unknown option $1" >&2; exit 2;;
  esac
done

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
app_dir="$PREFIX/$APP"
releases="$app_dir/releases"

if [[ "$SCOPE" == user ]]; then
  systemctl_cmd=(systemctl --user)
  unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
else
  systemctl_cmd=(systemctl)
  unit_dir=/etc/systemd/system
  if [[ $EUID -ne 0 && $systemd -eq 1 ]]; then
    echo "install.sh: system scope needs root (run with sudo, or --scope user)" >&2
    exit 1
  fi
fi

# switch_to points current at a release, remembering what it displaced so
# --rollback has somewhere to go back to.
switch_to() {
  local dest="$1" prev
  prev=$(readlink -f "$app_dir/current" 2>/dev/null || true)
  ln -sfn "$dest" "$app_dir/.current.new"
  mv -T "$app_dir/.current.new" "$app_dir/current"
  if [[ -n "$prev" && "$prev" != "$dest" ]]; then
    echo "$prev" > "$app_dir/previous"
  fi
}

if [[ $rollback -eq 1 ]]; then
  [[ -f "$app_dir/previous" ]] || { echo "install.sh: no previous release recorded" >&2; exit 1; }
  prev=$(cat "$app_dir/previous")
  [[ -d "$prev" ]] || { echo "install.sh: previous release $prev is gone" >&2; exit 1; }
  switch_to "$prev"
  echo "rolled back to $(basename "$prev")"
else
  dest="$releases/$VERSION"
  mkdir -p "$dest" "$app_dir"
  # A re-deploy of the same version overwrites in place rather than
  # accumulating. install.sh installs itself too: --rollback needs a copy on
  # the robot, and it is the same script whichever release it comes from.
  tar -cf - -C "$here" . | tar -xf - -C "$dest"
  chmod +x "$dest/bin/$APP"
  switch_to "$dest"
  echo "installed $APP $VERSION at $dest"
fi

if [[ $systemd -eq 1 ]]; then
  mkdir -p "$unit_dir"
  for u in "${UNITS[@]}"; do
    install -m 0644 "$app_dir/current/systemd/$u" "$unit_dir/$u"
  done
  # Units this release does not have must go: a renamed or removed node, or
  # a move between one unit per node and one for the app, would otherwise
  # leave a unit running against the new release's paths.
  shopt -s nullglob
  for existing in "$unit_dir/$APP.service" "$unit_dir/$APP"-*.service; do
    base=$(basename "$existing")
    keep=0
    for u in "${UNITS[@]}"; do
      [[ "$u" == "$base" ]] && keep=1
    done
    if [[ $keep -eq 0 ]]; then
      "${systemctl_cmd[@]}" disable --now "$base" >/dev/null 2>&1 || true
      rm -f "$existing"
      echo "removed stale unit $base"
    fi
  done
  shopt -u nullglob
  "${systemctl_cmd[@]}" daemon-reload
  "${systemctl_cmd[@]}" enable "$TARGET" >/dev/null 2>&1 || true
  if [[ $restart -eq 1 ]]; then
    "${systemctl_cmd[@]}" restart "$TARGET"
    echo "restarted $TARGET"
  fi
fi

# Keep the last $KEEP releases so a rollback target survives; never remove
# the one current points at, or the one it can roll back to.
if [[ $rollback -eq 0 && -d "$releases" ]]; then
  current=$(readlink -f "$app_dir/current" || true)
  previous=$(cat "$app_dir/previous" 2>/dev/null || true)
  # shellcheck disable=SC2012
  ls -1dt "$releases"/*/ 2>/dev/null | tail -n +$((KEEP + 1)) | while read -r old; do
    old=${old%/}
    resolved=$(readlink -f "$old")
    [[ "$resolved" == "$current" || "$resolved" == "$previous" ]] && continue
    rm -rf "$old"
    echo "pruned $(basename "$old")"
  done
fi
`

func shellQuote(s string) string {
	if s == "" {
		return "''"
	}
	for _, r := range s {
		safe := r == '-' || r == '_' || r == '.' || r == '/' || r == ':' ||
			(r >= 'a' && r <= 'z') || (r >= 'A' && r <= 'Z') || (r >= '0' && r <= '9')
		if !safe {
			return "'" + strings.ReplaceAll(s, "'", `'\''`) + "'"
		}
	}
	return s
}

func shellQuoteAll(ss []string) string {
	out := make([]string, len(ss))
	for i, s := range ss {
		out[i] = shellQuote(s)
	}
	return strings.Join(out, " ")
}
