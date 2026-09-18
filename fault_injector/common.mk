
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

NAME=fault_injector

# protocol.h (shared IPC message/pulse definitions) lives one level up.
EXTRA_INCVPATH+=$(PROJECT_ROOT)/../common


#This has to be included last
include $(MKFILES_ROOT)/qtargets.mk
