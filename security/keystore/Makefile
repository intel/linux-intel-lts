#
# Makefile - Intel Keystore Linux driver
#

# Keystore module
obj-$(CONFIG_KEYSTORE) := keystore.o

# Otherwise, select the full file list
keystore-$(CONFIG_KEYSTORE) +=\
	keystore_kmod.o\
	client.o \
	ioctls.o\
	mac.o\
	rdrand.o\
	api_dal.o\
	dal_client.o\
	applet_read.o\
	dal_context.o\
	dal_context_safe.o

keystore-$(CONFIG_KEYSTORE_DEBUG) +=\
	debug.o
