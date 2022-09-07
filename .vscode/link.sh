#! /bin/bash

SCRIPT_DIR=$(cd $(dirname $0); pwd)
WORKSPACE_DIR=$(cd "$SCRIPT_DIR/../../../"; pwd)"/.vscode"
echo "make symbolic link from $SCRIPT_DIR to $WORKSPACE_DIR"
ln -s $SCRIPT_DIR $WORKSPACE_DIR