include ../../../imports.mak

EXPORTS = ../../../exports

XDC = $(XDCROOT)/xdc

all: emt_src.zip

release: $(EXPORTS)/emt_src.zip

all clean emt_src.zip .make:
	@$(XDC) XDCPATH="^/..;$(TIDRIVERS.msp432);$(TIRTOS.msp432)" $@

$(EXPORTS)/emt_src.zip: emt_src.zip
	./rezip.ksh $< $@ emt_src/emt:emt emt_src:
