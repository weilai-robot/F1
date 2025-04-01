#!/bin/sh

# Only test on BASH and ZSH
if [ -n "$(echo $0 | grep bash)" ]; then
    cd "$(dirname "$BASH_SOURCE")"
    CUR_FILE=$(pwd)/$(basename "$BASH_SOURCE")
    CUR_DIR=$(dirname "$CUR_FILE")
    cd - >/dev/null
else
    echo "$0" | grep -q "$PWD"
    if [ $? -eq 0 ]; then
        CUR_FILE=$0
    else
        CUR_FILE=$(pwd)/$0
    fi
    CUR_DIR=$(dirname "$CUR_FILE")
fi

# Add PATH
if [ -z "$(echo $PATH | grep $CUR_DIR)" ]; then
    PATH=$CUR_DIR:$PATH
fi

# Auto-completion
complete -W 'unpack install upload help' pack