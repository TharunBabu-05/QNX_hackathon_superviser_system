
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

NAME=ultrasonic_task-1

# protocol.h (shared IPC message/pulse definitions) lives one level up,
# outside any single project.
EXTRA_INCVPATH+=$(PROJECT_ROOT)/../common

#This has to be included last
include $(MKFILES_ROOT)/qtargets.mk
