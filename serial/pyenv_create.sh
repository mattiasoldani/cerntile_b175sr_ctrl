#!/bin/bash

ENVNAME=serial_pyenv

echo "removing previously created environment..."
rm -rf "./$ENVNAME"
echo "done!"

echo "----------"

python3 -m venv $ENVNAME
source "./$ENVNAME/bin/activate"
pip install --upgrade pip
python3 -m pip install -r "./pyenv_req.txt"
deactivate

echo "----------"
echo "activate with:"
echo "source ./$ENVNAME/bin/activate"
