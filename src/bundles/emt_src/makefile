include ../../../imports.mak

EXPORTS = ../../../exports

TIDRIVERS ?= $(wildcard $(TIRTOS)/products/tidrivers_*)

TIRTOS_REPOS = $(TIRTOS)/packages;$(SYSBIOS)/packages;$(TIDRIVERS)/packages;$(EMTROOT)

XDC = $(XDCROOT)/xdc

all: emt_src.zip

release: $(EXPORTS)/emt_src.zip

all clean emt_src.zip .make:
	@$(XDC) XDCPATH="$(CURDIR)/../..;$(TIRTOS_REPOS)" $@

$(EXPORTS)/emt_src.zip: emt_src.zip
	./rezip.ksh $< $@ emt_src/emt:emt emt_src:
