set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EDGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_ROOT="${EDGE_ROOT}/config"
PTP_DIR="${CONFIG_ROOT}/ptp"
SYSCTL_FILE="${CONFIG_ROOT}/dds/sysctl_overlay.conf"
PTP4L_CFG="${PTP_DIR}/ptp4l_gm.cfg"
PHC2SYS_CFG="${PTP_DIR}/phc2sys.cfg"
FASTDDS_PROFILES="${CONFIG_ROOT}/dds/fastdds.xml"
CYCLONEDDS_URI="file://${CONFIG_ROOT}/dds/cyclonedds.xml"
PTP_INTERFACE="${PTP_INTERFACE:-eth0}"
DRY_RUN=0

usage() {
  cat <<USAGE
Usage: $0 [--dry-run] [--ptp-interface <iface>]
  --dry-run          Print the commands without running them (useful inside Docker).
  --ptp-interface    Network interface for ptp4l (default: ${PTP_INTERFACE}).
Environment overrides:
  PTP_INTERFACE      Same as --ptp-interface.
  CYCLONEDDS_URI     Override DDS XML path (defaults to ${CYCLONEDDS_URI}).
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --ptp-interface)
      shift
      [[ $# -gt 0 ]] || { echo "Missing value for --ptp-interface" >&2; exit 1; }
      PTP_INTERFACE="$1"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

run_cmd() {
  if [[ ${DRY_RUN} -eq 1 ]]; then
    echo "[dry-run] $*"
  else
    echo "+ $*"
    eval "$@"
  fi
}

if [[ ${DRY_RUN} -eq 0 && ${EUID} -ne 0 ]]; then
  echo "[WARN] This script should be run as root (or via sudo) on the bench rig." >&2
fi

if [[ ! -f "${SYSCTL_FILE}" ]]; then
  echo "[ERROR] Missing sysctl overlay at ${SYSCTL_FILE}" >&2
  exit 1
fi

run_cmd "sysctl -p ${SYSCTL_FILE}"

if [[ ! -f "${PTP4L_CFG}" || ! -f "${PHC2SYS_CFG}" ]]; then
  echo "[ERROR] Missing PTP configuration files under ${PTP_DIR}" >&2
  exit 1
fi

PTP4L_LOG="/tmp/aeris_ptp4l.log"
PHC2SYS_LOG="/tmp/aeris_phc2sys.log"

run_cmd "nohup ptp4l -f ${PTP4L_CFG} -i ${PTP_INTERFACE} > ${PTP4L_LOG} 2>&1 &"
run_cmd "nohup phc2sys -f ${PHC2SYS_CFG} > ${PHC2SYS_LOG} 2>&1 &"

echo "# Slave mode example (manually run if this rig is not the GM):" \
  && echo "#   sudo ptp4l -f ${PTP_DIR}/ptp4l_slave.cfg -i ${PTP_INTERFACE}"

printf '\n# DDS environment templates\n'
echo "export CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "# For Fast DDS:" \
     && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTDDS_PROFILES}" \
     && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"

printf '\n# TODO: Camera bring-up (e.g., gst-launch) goes here once the selected sensor is finalized.\n'
