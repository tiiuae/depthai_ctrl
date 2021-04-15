#!/bin/bash


# Abort script execution on errors
set -e

PROG="$(basename "$0")"

# Log everything to syslog.
exec 1> >(logger -s -t ${PROG}) 2>&1

DOMAIN="$1"
if [ -z "${DOMAIN}" ]; then
  echo "Missing libvirt domain parameter for ${PROG}." >&2
  exit 1
fi

IFS=: read VAR1 VAR2 <<< $(virsh dominfo ${DOMAIN} | grep State)
DOMAIN_STATE="$(echo -e "${VAR2}" | tr -d '[:space:]')"
if [ "${DOMAIN_STATE}" != 'running' ]; then
  echo "Domain ${DOMAIN} state (${DOMAIN_STATE}) is not running. Nothing to do to DepthAI camera." >&2
  exit 0
fi

if [ -z "${ACTION}" ]; then
  echo "Missing udev ACTION environment variable." >&2
  exit 1
fi

if [ "${ACTION}" == 'bind' ]; then
  COMMAND='attach-device'
elif [ "${ACTION}" == 'remove' ]; then
  COMMAND='detach-device'
  if [ "${PRODUCT}" == '3e7/2485/1' ]; then
    ID_VENDOR_ID=03e7
    ID_MODEL_ID=2485
  fi
  if [ "${PRODUCT}" == '3e7/f63b/100' ]; then
    ID_VENDOR_ID=03e7
    ID_MODEL_ID=f63b
  fi
else
  echo "Invalid udev ACTION: ${ACTION}" >&2
  exit 1
fi

DEVNUM=$((10#$DEVNUM))

#
# Now we have all the information we need to update the VM.
# Run the appropriate virsh-command, and ask it to read the
# update XML from stdin.
#
echo "Running virsh ${COMMAND} ${DOMAIN} for ${ID_VENDOR}." >&2
virsh "${COMMAND}" "${DOMAIN}" /dev/stdin <<END
<hostdev mode='subsystem' type='usb'>
  <source>
    <vendor id='0x${ID_VENDOR_ID}'/>
    <product id='0x${ID_MODEL_ID}'/>
  </source>
</hostdev>
END

exit 0
