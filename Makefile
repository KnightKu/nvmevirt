KERNELDIR := /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)
INSTALL_MOD_PATH :=

include Makefile.local

# Some distro kernel headers omit vendor certs (e.g. certs/rhel.pem),
# which breaks external module builds. If missing and not overridden,
# disable trusted/revocation key lists.
KBUILD_CERTS_ARGS :=
ifeq ($(wildcard $(KERNELDIR)/certs/rhel.pem),)
  ifeq ($(origin CONFIG_SYSTEM_TRUSTED_KEYS), undefined)
    KBUILD_CERTS_ARGS += CONFIG_SYSTEM_TRUSTED_KEYS=
  endif
  ifeq ($(origin CONFIG_SYSTEM_REVOCATION_KEYS), undefined)
    KBUILD_CERTS_ARGS += CONFIG_SYSTEM_REVOCATION_KEYS=
  endif
endif

default:
		$(MAKE) -C $(KERNELDIR) M=$(PWD) $(KBUILD_CERTS_ARGS) modules

install:
		$(MAKE) INSTALL_MOD_PATH="$(INSTALL_MOD_PATH)" -C $(KERNELDIR) $(KBUILD_CERTS_ARGS) modules_install

.PHONY: clean
clean:
	   $(MAKE) -C $(KERNELDIR) M=$(PWD) clean
	   rm -f cscope.out tags nvmev.S

.PHONY: cscope
cscope:
		cscope -b -R
		ctags *.[ch]

.PHONY: tags
tags: cscope

.PHONY: format
format:
	clang-format -i *.[ch]

.PHONY: dis
dis:
	objdump -d -S nvmev.ko > nvmev.S
