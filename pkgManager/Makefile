TARGET=pkgManager
CC=g++
CFLAGS=
LDFLAGS=-lprotobuf

SRC=./src
OBJ=./obj

SOURCES1 := $(wildcard $(SRC)/*.cpp)
OBJECTS1 := $(patsubst $(SRC)/%.cpp, $(OBJ)/%.o, $(SOURCES1))
SOURCES2 := $(wildcard $(SRC)/*.cc)
OBJECTS2 := $(patsubst $(SRC)/%.cc, $(OBJ)/%.o, $(SOURCES2))

SOURCES := $(SOURCES1) $(SOURCES2)
OBJECTS := $(OBJECTS1) $(OBJECTS2)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $^ $(LDFLAGS) -o $@

$(OBJ)/%.o: $(SRC)/%.cpp
	$(CC) -I$(SRC) $(CFLAGS) -c $< -o $@

$(OBJ)/%.o: $(SRC)/%.cc
	$(CC) -I$(SRC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET) 2

.PHONY: clean

clean:
	rm -f $(OBJ)/*.o $(TARGET)
