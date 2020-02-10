DEFINES+=APROS_PACKET_CONTROL_H=\"apros-packet-control.h\"

CONTIKI_PROJECT = ct_waveform_virnect

PLATFORMS_ONLY = apros

all: $(CONTIKI_PROJECT)

PROJECT_SOURCEFILES += apros-packet-control.c

CONTIKI = ../../..

MAKE_MAC = MAKE_MAC_CSMA
MAKE_NET = MAKE_NET_NULLNET
MAKE_ROUTING = MAKE_ROUTING_NULLROUTING

CONTIKI_WITH_RIME = 1

include $(CONTIKI)/Makefile.dir-variables
include $(CONTIKI)/Makefile.include
