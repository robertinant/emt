# define default set of platforms, compiler toolchains, and tests to build
PLATFORMS ?= cc13xx cc26xx msp432 cc3200 

ifeq (,$(TESTS))
    TESTS        = blink hello alloc string 
    TESTS.cc3200 = zumo wifi pstring MultiBlink 
    TESTS.msp432 = pstring MultiBlink 
    TESTS.cc26xx = pstring MultiBlink 
endif

# define (optional) tools to run on output files to summarize footprint
XDCROOT ?= $(wildcard $(TREES)/xdcprod/xdcprod-t67/product/Linux/xdctools*)
OBJTOOL  = $(if $(XDCROOT),$(XDCROOT)/xs -c $(CURDIR)/objsum.xs,:)
MAPTOOL  = $(if $(XDCROOT),$(XDCROOT)/xs -c $(CURDIR)/mapsum.xs,:)

export XDCROOT
export OBJTOOL
export MAPTOOL

# define top-level build command
define BUILD
    @echo ======== $1 ========
    +$(MAKE) --no-print-directory -C $1 $2 $3

endef

define BUILD_ALL
	@echo building '$@' for all platforms and tests ...
	$(foreach p,$(PLATFORMS), $(call BUILD, $p, $@,))
	$(foreach p,$(PLATFORMS), \
            $(foreach t,$(TESTS) $(TESTS.$p), \
                $(call BUILD, $p/tests/$t, $@,)))

endef

TREE_ROOT = $(firstword $(subst /src/, /src/,$(CURDIR)))
TREE_NAME = $(notdir $(TREE_ROOT))

ifeq (,$(TIRTOS))
    $(warning "warning: importing ../../products.mak to get TIRTOS definition")
    include ../../products.mak
endif

ZUMA_TREE = $(firstword $(subst /, ,$(lastword $(subst /zumaprod/, ,$(TIRTOS)))))
ifeq (,$(ZUMA_TREE))
    $(error "can't compute ZUMA_TREE from TIRTOS = '$(TIRTOS)'")
endif

# ensure "all" is the default build goal
all: makefile

# rule for the "universal closure" (required by Energia product build)
CLOSURE_ZIPS = $(foreach plat,$(PLATFORMS),$(plat)/closure.zip)
closure-$(TREE_NAME).tar.gz: $(TREE_ROOT)/exports/emt_src.zip $(CLOSURE_ZIPS)
	@echo making $@ ...
	rm -f $@
	./copy.ksh $<

# rules to make the loadable board .zip archives
.PHONY: board.msp432 board.cc3200 board.cc13xx
boards: board.msp432 board.cc3200 board.cc13xx

board.cc13xx: cc13xx-$(TREE_NAME).zip
cc13xx-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip cc13xx/closure.zip
	@echo making $@ ...
	./cc13xx/mkbrd.ksh $< $(ZUMA_TREE)

board.msp432: msp432-$(TREE_NAME).zip
msp432-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip msp432/closure.zip
	@echo making $@ ...
	./msp432/mkbrd.ksh $< $(ZUMA_TREE)

board.cc3200: cc3200-$(TREE_NAME).zip
cc3200-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip cc3200/closure.zip
	@echo making $@ ...
	./cc3200/mkbrd.ksh $< $(ZUMA_TREE)

# rule for device-specific closure zips; used to create the "universal closure"
%/closure.zip: | all
	@echo making $@ "[ $(TREE_NAME) ]" ...
	@echo "version: $(TREE_NAME)" > $*/closure/version.txt
	@echo "source : $(TREE_ROOT)" >> $*/closure/version.txt
	@cd $* && $(XDCROOT)/bin/zip -rq closure.zip closure

# rule to build the specified goal for all platforms, compilers, and tests
%:
	$(call BUILD_ALL,$@)

clean:
	-rm -f closure-$(TREE_NAME).tar.gz
	-rm -f msp432-$(TREE_NAME).zip
	-rm -f cc3200-$(TREE_NAME).zip
	-rm -f cc13xx-$(TREE_NAME).zip
	$(call BUILD_ALL,clean)
