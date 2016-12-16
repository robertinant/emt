include ../../../imports.mak

EXPORTS = ../../../exports

XDC = $(XDCROOT)/xdc

# Choose just one core SDK: these repos are only used to compute compatibility
TIRTOS_REPOS = $(TIDRIVERS.msp432);$(TIRTOS.msp432)

all: emt_src.zip

release: $(EXPORTS)/emt_src.zip

all clean emt_src.zip .make:
	@$(XDC) XDCPATH="^/..;$(TIRTOS_REPOS)" $@

$(EXPORTS)/emt_src.zip: emt_src.zip
	./rezip.ksh $< $@ emt_src/emt:emt emt_src:
