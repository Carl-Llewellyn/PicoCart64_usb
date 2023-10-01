#!/bin/sh

set -e

help()
{
    echo "Usage: $0 [ -c ]"
    echo "       -c: Deletes the backup files generated by the indent tool"
    exit 1
}

while getopts ch name
do
  case $name in
    c)
      CLEANUP=1
      ;;
    ?)
      help
  esac
done



# Hacky but apparently reliable way to get the absolute path of the script directory
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

C_DIRECTORIES="
    n64
    n64_cic
    picocart64_v1
    picocart64_shared
    stdio_async_uart
"

for x in $C_DIRECTORIES; do
    C_SOURCES=$(find "$SCRIPTPATH/../$x" -type f -name '*.c')
    H_SOURCES=$(find "$SCRIPTPATH/../$x" -type f -name '*.h' ! -name 'rom.h')

    SIMPLE_BACKUP_SUFFIX="~picocart64~" indent $C_SOURCES $H_SOURCES -nbad -bap -bbo -hnl -br -brs -c33 -cd33 -ncdb -ce -ci4 -cli0 -d0 -di1 -nfc1 -i4 -ip0 -l180 -lp -npcs -nprs -npsl -sai -saf -saw -ncs -nsc -sob -nfca -cp33  -ss -ts4 -il1

    if [[ $CLEANUP ]]; then
        # Delete backup files with '~picocart64~' suffix
        find "$SCRIPTPATH/../$x" -type f -name '*~picocart64~' -exec rm -f {} ';'
    fi
done
