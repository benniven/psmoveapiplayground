
# Set the path to the PS Move API source directory
# (there should be a build inside "build/" in that directory)
PSMOVEAPI_ROOT := ../psmoveapi/

TARGET := playground

PKGS := opencv

OBJS := $(patsubst %.c,%.o,$(wildcard *.c))

CFLAGS := $(shell pkg-config --cflags $(PKGS)) -I$(PSMOVEAPI_ROOT)
LDFLAGS := $(shell pkg-config --libs $(PKGS)) -L$(PSMOVEAPI_ROOT)/build/ -lpsmoveapi

all: $(TARGET)

run: $(TARGET)
	PATH=$$PATH:. LD_LIBRARY_PATH=$(PSMOVEAPI_ROOT)/build/ $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $(TARGET) $(OBJS) $(LDFLAGS)

clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: all run clean
.DEFAULT: all

