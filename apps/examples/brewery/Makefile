include $(APPDIR)/Make.defs

CONFIG_EXAMPLES_BREWERY_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_BREWERY_STACKSIZE ?= 2048

APPNAME = brewery
PRIORITY = $(CONFIG_EXAMPLES_BREWERY_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_BREWERY_STACKSIZE)

ASRCS = 
CSRCS =
MAINSRC = brewery_main.c

CONFIG_EXAMPLES_BREWERY_PROGNAME ?= brewery$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_BREWERY_PROGNAME)

include $(APPDIR)/Application.mk
