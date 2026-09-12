# Every immediate harness directory supplies check and clean targets.
HOST_DIRS := $(patsubst %/Makefile,%,$(wildcard */Makefile))
CHECK_TARGETS := $(addsuffix /check,$(HOST_DIRS))
CLEAN_TARGETS := $(addsuffix /clean,$(HOST_DIRS))

.PHONY: check clean $(CHECK_TARGETS) $(CLEAN_TARGETS)

check: $(CHECK_TARGETS)

clean: $(CLEAN_TARGETS)

$(CHECK_TARGETS):
	$(MAKE) -C $(@D) check

$(CLEAN_TARGETS):
	$(MAKE) -C $(@D) clean
