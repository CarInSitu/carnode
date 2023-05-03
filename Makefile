.PHONY: build
build:
	pio run

lint:
	clang-format -i `find src lib include -name '*.cpp' -or -name '*.h' -or -name '*.ino' | xargs`

upload: upload_ota

upload_serial:
	pio run -t upload

upload_ota:
	@./upload_ota.sh

upload_ota_all:
	@./upload_ota.sh all

clean:
	pio run -t clean

monitor:
	pio device monitor
