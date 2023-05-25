CC = gcc
CFLAGS = -Wall -Werror

# Path to CUnit headers and libraries
CUNIT_INCLUDE = -I/path/to/cunit/include
CUNIT_LIBS = -L/path/to/cunit/lib -lcunit

# Source file and test executable name
SOURCE = bufHandlerTest.c
TARGET = bufHandlerTest

all: $(TARGET)

$(TARGET): $(SOURCE)
	$(CC) $(CFLAGS) $(CUNIT_INCLUDE) -o $@ $< $(CUNIT_LIBS)

clean:
	rm -f $(TARGET)