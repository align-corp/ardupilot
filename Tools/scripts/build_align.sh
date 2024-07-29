#!/bin/bash
# build for all copters boards

set -e

# specify boards
BOARDS="AP3-M460 AP3-M460-dev AP3-M490 AP3-M490-dev "
BOARDS+="AP6-M460 AP6-M460-dev AP6-M460-G3P AP6-M490 AP6-M490-dev AP6-M490-G3P "
BOARDS+="AP6-M6T22 AP6-M6T22-dev AP6-M4T12-dev AP6-M4T12 "
BOARDS+="AP6v2-M460 AP6v2-M460-dev AP6v2-M460-G3P AP6v2-M490 AP6v2-M490-dev AP6v2-M490-G3P "
BOARDS+="AP6v2-M6T22 AP6v2-M6T22-dev AP6v2-M4T12-dev AP6v2-M4T12 "

# prepare align-build folder
if [ -d "align-build" ]; then
    rm -rf align-build
fi
mkdir align-build

# compile
for b in $BOARDS; do
    echo "Building: $b"
    ./waf configure --board $b
    ./waf clean
    ./waf copter
    cp build/$b/bin/arducopter.apj align-build/$b.apj
done

echo "align builds completed"

exit 0
