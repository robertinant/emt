# define default set of platforms, compiler toolchains, and tests to build
PLATFORMS ?= cc13x2 cc13xx msp432 msp432p4111 msp432e cc32xx

ifeq (,$(TESTS))
    TESTS        = blink hello alloc string
    TESTS.cc32xx = pstring MultiBlink
    TESTS.msp432 = pstring MultiBlink
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

# import SDK locations from imports.mak
include $(TREE_ROOT)/imports.mak

# ensure "all" is the default build goal
all: makefile

# rule for the "universal closure" (required by Energia product build)
CLOSURE_ZIPS = $(foreach plat,$(PLATFORMS),$(plat)/closure.zip)
closure-$(TREE_NAME).tar.gz: $(TREE_ROOT)/exports/emt_src.zip $(CLOSURE_ZIPS)
	@echo making $@ ...
	rm -f $@
	./copy.ksh $<

# rules to make the loadable board .zip archives
.PHONY: boards board.msp432 board.msp432p4111 board.cc32xx board.cc13xx board.cc13x2

boards: $(foreach p, $(PLATFORMS), board.$(p))
	@echo $^ done

board.cc13xx: cc13xx-$(TREE_NAME).zip
cc13xx-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip cc13xx/closure.zip
	@echo making $@ ...
	./cc13xx/mkbrd.ksh $< $(SDK.cc13xx) $(dir $(filter %/closure.zip,$^))

board.cc13x2: cc13x2-$(TREE_NAME).zip
cc13x2-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip cc13x2/closure.zip
	@echo making $@ ...
	./cc13x2/mkbrd.ksh $< $(SDK.cc13x2) $(dir $(filter %/closure.zip,$^))

board.msp432: msp432-$(TREE_NAME).zip
msp432-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip msp432/closure.zip
	@echo making $@ ...
	./msp432/mkbrd.ksh $< $(SDK.msp432) $(dir $(filter %/closure.zip,$^))

board.msp432p4111: msp432p4111-$(TREE_NAME).zip
msp432p4111-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip msp432p4111/closure.zip
	@echo making $@ ...
	./msp432p4111/mkbrd.ksh $< $(SDK.msp432) $(dir $(filter %/closure.zip,$^))

board.msp432e: msp432e-$(TREE_NAME).zip
msp432e-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip msp432e/closure.zip
	@echo making $@ ...
	./msp432e/mkbrd.ksh $< $(SDK.msp432e) $(dir $(filter %/closure.zip,$^))

board.cc32xx: cc32xx-$(TREE_NAME).zip
cc32xx-$(TREE_NAME).zip: $(TREE_ROOT)/exports/emt_src.zip cc32xx/closure.zip
	@echo making $@ ...
	./cc32xx/mkbrd.ksh $< $(SDK.cc32xx) $(dir $(filter %/closure.zip,$^))

# rule for device-specific closure zips; used to create board packages
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
	-rm -f msp432e-$(TREE_NAME).zip
	-rm -f cc32xx-$(TREE_NAME).zip
	-rm -f cc13xx-$(TREE_NAME).zip
	-rm -f cc13x2-$(TREE_NAME).zip
	$(call BUILD_ALL,clean)
