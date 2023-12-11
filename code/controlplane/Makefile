SHELL:=/bin/bash
.DEFAULT_GOAL := help

# STM32Programmer CLI locations from installer at
# https://www.st.com/en/development-tools/stm32cubeprog.html
export PATH := $(HOME)/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin:$(PATH)

TARGET_PATH := target/thumbv7em-none-eabi/release
BIN_PATH := $(TARGET_PATH)/xserve-raid-controlplane.bin

##
### main targets
##

.PHONY: $(BIN_PATH) # so that rust can decide if rebuild is needed, not make
$(BIN_PATH):
	cargo objcopy --release -- -O binary $@

.PHONY: build
build: $(BIN_PATH) ## Build the release binary

.PHONY: flash
flash: build ## flash the release binary to a device in DFU mode
	STM32_Programmer.sh -c port=USB1 -q -d $(BIN_PATH) 0x8000000 -s 0x8000000

##
### help
##

.PHONY: help
help: ## Dislay this help
	@IFS=$$'\n'; for line in `grep -h -E '^[a-zA-Z_#-]+:?.*?## .*$$' $(MAKEFILE_LIST)`; do if [ "$${line:0:2}" = "##" ]; then \
	echo $$line | awk 'BEGIN {FS = "## "}; {printf "\n\033[33m%s\033[0m\n", $$2}'; else \
	echo $$line | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'; fi; \
	done; unset IFS;