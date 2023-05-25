CC = gcc
CFLAGS = -Wall

CUNIT_LIBS = -lcunit

# Source file and test executable name
SOURCE := bufHandlerTest.c ../PSE_STM_Communication/Core/Src/bufHandler.c
TARGET := bufHandlerTest
INCDIR := ../PSE_STM_Communication/Core/Inc
DFLAGS := CUNIT_TEST


all: $(TARGET)

$(TARGET): $(SOURCE) $(INCDIR)/bufHandler.h
	$(CC) $^ -o $@ $(CUNIT_LIBS) -I $(INCDIR) -D$(DFLAGS) $(CFLAGS) 

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)