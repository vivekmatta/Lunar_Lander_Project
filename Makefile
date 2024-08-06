export PATH := /home/shay/a/ece270/bin:$(PATH)
export LD_LIBRARY_PATH := /home/shay/a/ece270/lib:$(LD_LIBRARY_PATH)

YOSYS=yosys
NEXTPNR=nextpnr-ice40
SHELL=bash

PROJ	= lab12
PINMAP 	= pinmap.pcf
TCLPREF = addwave.gtkw
SRC	    = top.sv
ICE   	= ice40hx8k.sv
CHK 	= check.bin
DEM 	= demo.bin
JSON    = ll.json
SUP     = support/cells_*.v
UART	= uart/uart.v uart/uart_tx.v uart/uart_rx.v
FILES   = $(ICE) $(SRC) $(UART)
TRACE	= $(PROJ).vcd
BUILD   = ./build

DEVICE  = 8k
TIMEDEV = hx8k
FOOTPRINT = ct256

all: cram

#########################
# Flash to FPGA
$(BUILD)/$(PROJ).json : $(ICE) $(SRC) $(PINMAP) Makefile
	# lint with Verilator
	verilator --lint-only --top-module top $(SRC)
	# if build folder doesn't exist, create it
	mkdir -p $(BUILD)
	# synthesize using Yosys
	$(YOSYS) -p "read_verilog -sv -noblackbox $(FILES); synth_ice40 -top ice40hx8k -json $(BUILD)/$(PROJ).json"

$(BUILD)/$(PROJ).asc : $(BUILD)/$(PROJ).json
	# Place and route using nextpnr
	$(NEXTPNR) --hx8k --package ct256 --pcf $(PINMAP) --asc $(BUILD)/$(PROJ).asc --json $(BUILD)/$(PROJ).json 2> >(sed -e 's/^.* 0 errors$$//' -e '/^Info:/d' -e '/^[ ]*$$/d' 1>&2)

$(BUILD)/$(PROJ).bin : $(BUILD)/$(PROJ).asc
	# Convert to bitstream using IcePack
	icepack $(BUILD)/$(PROJ).asc $(BUILD)/$(PROJ).bin

#########################
# Verification Suite
VFLAGS = --build --cc --exe --trace-fst --Mdir build

verify_ll_display: top.sv tb.cpp
	verilator --lint-only -Wno-MULTITOP top.sv
	@echo ========================================
	@echo Compiling and verifying ll_display...
	@rm -rf build
	verilator $(VFLAGS) --top-module ll_display -CFLAGS -DLL_DISPLAY top.sv tb.cpp 1>/dev/null
	./build/Vll_display

verify_lunarlander: top.sv tb_ll.sv
	verilator --lint-only -Wno-MULTITOP top.sv
	@echo ========================================
	@echo Compiling and verifying lunarlander...
	@rm -rf build
	@mkdir -p build
	yosys -p "read_verilog -sv -noblackbox $(FILES); synth_ice40 -top lunarlander; write_verilog build/ll.v"
	iverilog -g2012 tb_ll.sv build/ll.v $(SUP) -o build/sim
	./build/sim

view_ll_display: verify_ll_display
	gtkwave gtkw/ll_display.gtkw

view_lunarlander: verify_lunarlander
	gtkwave gtkw/lunarlander.gtkw
	
#########################
# ice40 Specific Targets
check: $(CHK)
	iceprog -S $(CHK)
	
demo:  $(DEM)
	iceprog -S $(DEM)

flash: $(BUILD)/$(PROJ).bin
	iceprog $(BUILD)/$(PROJ).bin

cram: $(BUILD)/$(PROJ).bin
	iceprog -S $(BUILD)/$(PROJ).bin

time: $(BUILD)/$(PROJ).asc
	icetime -p $(PINMAP) -P $(FOOTPRINT) -d $(TIMEDEV) $<

#########################
# Clean Up
clean:
	rm -rf build/ *.fst *.vcd verilog.log abc.history