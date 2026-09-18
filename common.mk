
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

NAME=hello



#This has to be included last
include $(MKFILES_ROOT)/qtargets.mk
