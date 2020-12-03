#!/bin/sh

# Source this for testing on macOS.

setButton() {
    echo >> /var/tmp/button.0
    pushButton "$1"
    rm /var/tmp/button.0
}

pushButton() {
    BUTTON="$1"
    echo >> /var/tmp/button."$BUTTON"
    sleep 1
    rm /var/tmp/button."$BUTTON"
}

pan() {
    handleAxis "$1" "$2" 1
}

tilt() {
    handleAxis "$1" "$2" 2
}

zoom() {
    handleAxis "$1" "$2" 3
}

handleAxis() {
    SPEED="$1"
    TIME="$2"
    AXIS="$3"

    echo "$SPEED" >> /var/tmp/axis."$AXIS"
    sleep "$TIME"
    rm /var/tmp/axis."$AXIS"
}
