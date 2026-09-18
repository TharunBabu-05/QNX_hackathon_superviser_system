
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

NAME=oled_task

# protocol.h (shared IPC message/pulse definitions) lives one level up.
EXTRA_INCVPATH+=$(PROJECT_ROOT)/../common


#This has to be included last
include $(MKFILES_ROOT)/qtargets.mk
