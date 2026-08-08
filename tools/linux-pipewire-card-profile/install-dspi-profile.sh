#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

PROFILE_NAME="dspi-iec958.conf"
PATH_NAME="dspi-iec958-output.conf"
RULE_NAME="91-dspi-pipewire-alsa.rules"

PROFILE_SRC="${SCRIPT_DIR}/${PROFILE_NAME}"
PATH_SRC="${SCRIPT_DIR}/${PATH_NAME}"
RULE_SRC="${SCRIPT_DIR}/${RULE_NAME}"

NO_RESTART=0
DRY_RUN=0

usage() {
    cat <<EOF
Usage: $(basename "$0") [--no-restart] [--dry-run]

Installs the local DSPi ACP profile, mixer path, and udev rule.

This profile targets PipeWire: it uses PipeWire's ACP mechanism and does not
take effect under the classic PulseAudio daemon (which uses a different udev
property and config directory). PipeWire is the default on current Fedora,
Ubuntu, Debian and Arch.

The ACP profile/mixer-path files are installed under your user config
directory (\$XDG_CONFIG_HOME/alsa-card-profile/mixer, no root needed) since
PipeWire's ACP loader checks there in addition to /usr/share, and /usr/share
is read-only on ostree/immutable systems like Fedora Kinoite/Silverblue.

The udev rule and udev reload/trigger need the real host. If this script
is run inside a toolbox (detected via /run/.toolboxenv), those steps are
relayed to the host via flatpak-spawn --host.

Options:
  --no-restart   Do not restart user audio services after install.
  --dry-run      Print what would be done without changing the system.
  -h, --help     Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-restart) NO_RESTART=1 ;;
        --dry-run) DRY_RUN=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

IN_TOOLBOX=0
[[ -f /run/.toolboxenv ]] && IN_TOOLBOX=1

need_cmd() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "missing required command: $1" >&2
        exit 1
    fi
}

need_host_cmd() {
    local check_cmd="$1"
    # In a toolbox, root escalation goes through pkexec instead of sudo
    # (see flush_host below), so check for that instead.
    [[ "${IN_TOOLBOX}" -eq 1 && "$1" == "sudo" ]] && check_cmd="pkexec"
    if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
        if ! flatpak-spawn --host -- command -v "${check_cmd}" >/dev/null 2>&1; then
            echo "missing required command on host: ${check_cmd}" >&2
            exit 1
        fi
    else
        need_cmd "${check_cmd}"
    fi
}

run() {
    printf '+'
    printf ' %q' "$@"
    printf '\n'
    if [[ "${DRY_RUN}" -eq 0 ]]; then
        "$@"
    fi
}

# Route an unprivileged command to the host when running inside a toolbox
# (pactl/systemctl talk to the per-user audio/session bus that toolbox shares
# with the host). Outside a toolbox it just runs locally.
run_user_host() {
    if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
        flatpak-spawn --host -- "$@"
    else
        "$@"
    fi
}

# True if COMMAND exists where unprivileged host commands run (the host when in
# a toolbox, otherwise the local environment).
host_has_cmd() {
    if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
        flatpak-spawn --host -- command -v "$1" >/dev/null 2>&1
    else
        command -v "$1" >/dev/null 2>&1
    fi
}

# Privileged host operations: udev lives on the real host, not the container's
# isolated view of /etc and /run/udev. Rather than elevate once per command
# (each pkexec/sudo call can trigger its own auth prompt), queue them with
# queue_host and run the whole batch in a single elevated shell via flush_host.
#
# In a toolbox, escalation uses pkexec (polkit GUI agent) instead of sudo:
# flatpak-spawn --host relays stdio as pipes, so sudo can't prompt for a
# password there ("a terminal is required to read the password"), whereas
# pkexec authenticates via the session's polkit agent.
HOST_CMDS=()

queue_host() {
    HOST_CMDS+=("$(printf '%q ' "$@")")
    printf '+'
    [[ "${IN_TOOLBOX}" -eq 1 ]] && printf ' [host]'
    printf ' %q' "$@"
    printf '\n'
}

flush_host() {
    [[ "${#HOST_CMDS[@]}" -eq 0 ]] && return 0
    local script
    script="set -e; $(printf '%s; ' "${HOST_CMDS[@]}")"
    if [[ "${DRY_RUN}" -eq 0 ]]; then
        if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
            echo "Applying host udev changes (authenticate once via polkit)..."
            flatpak-spawn --host -- pkexec /bin/sh -c "${script}"
        else
            sudo /bin/sh -c "${script}"
        fi
    fi
    HOST_CMDS=()
}

audio_service_state() {
    if command -v systemctl >/dev/null 2>&1; then
        if systemctl --user --quiet is-active wireplumber 2>/dev/null ||
           systemctl --user --quiet is-active pipewire 2>/dev/null; then
            echo "pipewire"
            return
        fi
    fi
    if host_has_cmd pactl && run_user_host pactl info 2>/dev/null | grep -qi '^Server Name:.*PulseAudio'; then
        echo "pulseaudio"
        return
    fi
    echo "unknown"
}

print_dspi_summary() {
    if ! host_has_cmd pactl; then
        echo "pactl not found; skipping PulseAudio/PipeWire summary."
        return
    fi
    if ! run_user_host pactl info >/dev/null 2>&1; then
        echo "pactl cannot connect to the current audio server; skipping summary."
        return
    fi

    echo
    echo "DSPi cards:"
    run_user_host pactl list cards short | grep -F 'RYNLABS_RYNLABS_DSPi' || echo "  (none visible)"
    echo
    echo "DSPi sinks:"
    run_user_host pactl list sinks short | grep -F 'RYNLABS_RYNLABS_DSPi' || echo "  (none visible)"
}

retrigger_dspi_cards() {
    local found=0
    local card real

    [[ -d /sys/class/sound ]] || return 0
    for card in /sys/class/sound/card*; do
        [[ -e "${card}/id" ]] || continue
        [[ "$(cat "${card}/id")" == "DSPi" ]] || continue
        real="$(readlink -f "${card}")"
        found=1
        queue_host udevadm trigger --action=change "${real}"
    done

    if [[ "${found}" -eq 0 ]]; then
        echo "DSPi sound card is not currently present; install will apply after replug/reboot."
    fi
}

restart_audio() {
    local server
    server="$(audio_service_state)"

    if [[ "${NO_RESTART}" -eq 1 ]]; then
        echo "Skipping audio service restart (--no-restart). Replug DSPi or restart audio manually."
        return 0
    fi

    case "${server}" in
        pipewire)
            if ! command -v systemctl >/dev/null 2>&1; then
                echo "systemctl not found; restart PipeWire manually or re-login."
                return 0
            fi
            echo "Restarting user PipeWire services..."
            run systemctl --user restart wireplumber pipewire pipewire-pulse
            ;;
        pulseaudio)
            echo "NOTE: classic PulseAudio detected. The DSPi profile uses PipeWire's"
            echo "ACP mechanism (ACP_PROFILE_SET + ~/.config/alsa-card-profile) and will"
            echo "NOT take effect under the classic PulseAudio daemon; consider switching"
            echo "to PipeWire. Restarting PulseAudio for completeness..."
            if command -v systemctl >/dev/null 2>&1 &&
               systemctl --user list-unit-files pulseaudio.service >/dev/null 2>&1; then
                run systemctl --user restart pulseaudio.service
            else
                run pulseaudio -k
            fi
            ;;
        *)
            echo "Could not detect PipeWire or PulseAudio. Replug DSPi or restart your audio session manually."
            ;;
    esac
}

for src in "${PROFILE_SRC}" "${PATH_SRC}" "${RULE_SRC}"; do
    if [[ ! -f "${src}" ]]; then
        echo "missing required file: ${src}" >&2
        exit 1
    fi
done

if [[ "${IN_TOOLBOX}" -eq 1 ]]; then
    need_cmd flatpak-spawn
    echo "Detected toolbox environment; udev steps will run on the host via flatpak-spawn."
    if ! flatpak-spawn --host -- test -f "${RULE_SRC}"; then
        echo "error: ${RULE_SRC} is not visible from the host." >&2
        echo "Run this installer from a location under \$HOME so the toolbox and host share the path." >&2
        exit 1
    fi
fi

need_cmd install
need_host_cmd sudo
need_host_cmd udevadm

CONFIG_HOME="${XDG_CONFIG_HOME:-${HOME}/.config}"
PROFILE_DIR="${CONFIG_HOME}/alsa-card-profile/mixer/profile-sets"
PATH_DIR="${CONFIG_HOME}/alsa-card-profile/mixer/paths"

PROFILE_DST="${PROFILE_DIR}/${PROFILE_NAME}"
PATH_DST="${PATH_DIR}/${PATH_NAME}"
RULE_DST="/etc/udev/rules.d/${RULE_NAME}"

echo "Profile directory: ${PROFILE_DIR}"
echo "Mixer path directory: ${PATH_DIR}"
print_dspi_summary

echo
echo "Installing DSPi ACP profile and mixer path (user config, no root needed)..."
run mkdir -p "${PROFILE_DIR}" "${PATH_DIR}"
run install -m 0644 "${PROFILE_SRC}" "${PROFILE_DST}"
run install -m 0644 "${PATH_SRC}" "${PATH_DST}"

echo "Installing DSPi udev rule..."
queue_host install -m 0644 "${RULE_SRC}" "${RULE_DST}"

echo "Reloading udev rules..."
queue_host udevadm control --reload-rules

echo "Retriggering DSPi sound card if present..."
retrigger_dspi_cards

flush_host

restart_audio

echo
echo "Installed DSPi IEC958 profile."
print_dspi_summary
echo
echo "Expected result: one DSPi profile named Digital Stereo (IEC958), with HW_VOLUME_CTRL on the DSPi sink."
