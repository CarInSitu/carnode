#!/bin/bash

if [ "$1" == "all" ]
then
	echo "Upload to all detected"
	CARS=$(avahi-browse -t _arduino._tcp | grep CarNode | awk -F' ' '{ print $4 }')
	[ "$CARS" == "" ] && (echo "No car found."; exit 1) || echo "Cars: $CARS"
	for car in $CARS
	do
		PLATFORMIO_UPLOAD_FLAGS=--auth=CarInSituOTA pio run --target upload --upload-port ${car}.local
	done

else
	if [ "${OTA_TARGET}" == "" ]
	then
		echo "No OTA_TARGET environment variable."
		exit 1
	else
		echo Upload to ${OTA_TARGET}
	fi
	PLATFORMIO_UPLOAD_FLAGS=--auth=CarInSituOTA pio run -t upload --upload-port ${OTA_TARGET}.local
fi
