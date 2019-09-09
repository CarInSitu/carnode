build:
	pio run

lint:
	clang-format -i `find src lib include -name '*.cpp' -or -name '*.h' -or -name '*.ino' | xargs`
