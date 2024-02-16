CC := gcc
CFLAGS := -ggdb3 -Iinclude -Isubmodules
WARNINGS := -Wall -Wpedantic -Wextra
WARNINGS += -Wdeclaration-after-statement    # style
WARNINGS += -Wno-missing-field-initializers  # cvar issues
LDFLAGS := -lm

# add SDL2
CFLAGS += $(shell pkg-config --cflags sdl2)
LDFLAGS += $(shell pkg-config --libs sdl2)
# add zlib
CFLAGS += $(shell pkg-config --cflags zlib)
LDFLAGS += $(shell pkg-config --libs zlib)
# add libbsd
CFLAGS += $(shell pkg-config --cflags libbsd)
LDFLAGS += $(shell pkg-config --libs libbsd)

SRC_DIR := src
OBJ_DIR := obj
BUILD_DIR := build
ASSET_DIR := assets
TARGET := $(BUILD_DIR)/b173c

CFLAGS += -I$(SRC_DIR)

SRC_FILES := $(shell find $(SRC_DIR)/ -type f -name "*.c")
HDR_FILES := $(shell find $(SRC_DIR)/ -type f -name "*.h")
OBJ_FILES := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC_FILES))

PY := python

SHADER_PREP := src/vid/shaders/prepare_shader_sources.py
SHADER_SOURCES := $(OBJ_DIR)/gl_shader_sources.c
SHADER_SOURCES_OBJ := $(OBJ_DIR)/gl_shader_sources.o
SHADER_FILES := $(wildcard $(SRC_DIR)/vid/shaders/*.glsl)
OBJ_FILES += $(SHADER_SOURCES_OBJ)

ASSET_PREP := assets/prepare_asset_sources.py
ASSET_SOURCES := $(OBJ_DIR)/asset_sources.c
ASSET_SOURCES_OBJ := $(OBJ_DIR)/asset_sources.o
ASSET_FILES := $(shell find $(ASSET_DIR)/ -type f -name "*.png") # xd
OBJ_FILES += $(ASSET_SOURCES_OBJ)

# submodule stuff
# {
DEP_HASHMAP := submodules/hashmap.c/hashmap.c
DEP_STB_IMAGE := submodules/stb/stb_image.h
DEPS_C := $(DEP_HASHMAP)
DEPS_H := $(DEP_STB_IMAGE)
DEPS := $(DEPS_C) $(DEPS_H)

SRC_FILES += $(DEPS_C)
OBJ_FILES += $(patsubst %.c,$(OBJ_DIR)/%.o,$(DEPS_C))

$(DEPS):
	git submodule update --init --recursive --remote # someone forgot the submodules :-P

$(OBJ_DIR)/%.o: %.c $(HDR_FILES) $(DEPS)
	./mkdirs.sh $@
	$(CC) -c $(CFLAGS) -o $@ $<
# }

all: $(OBJ_DIR) $(BUILD_DIR) $(TARGET)

clean:
	rm $(OBJ_DIR) -fr
	rm $(TARGET) -fr

$(OBJ_DIR):
	mkdir $(OBJ_DIR)

$(BUILD_DIR):
	mkdir $(BUILD_DIR)


# dont care about warnings from src/ext/
$(OBJ_DIR)/ext/%.o: $(SRC_DIR)/ext/%.c $(HDR_FILES)
	./mkdirs.sh $@
	$(CC) -c $(CFLAGS) -o $@ $< $(LDFLAGS)


# project sources (c)
# {
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c $(HDR_FILES) $(DEPS)
	./mkdirs.sh $@
	$(CC) -c $(CFLAGS) $(WARNINGS) -o $@ $< $(LDFLAGS)
# }

# project sources (glsl)
# {
$(SHADER_SOURCES): $(SHADER_FILES)
	$(PY) $(SHADER_PREP) $@ $^

$(SHADER_SOURCES_OBJ): $(SHADER_SOURCES)
	$(CC) -c $(CFLAGS) -o $@ $< $(LDFLAGS)
# }

# assets
# {
$(ASSET_SOURCES): $(ASSET_FILES)
	$(PY) $(ASSET_PREP) $@ $^

$(ASSET_SOURCES_OBJ): $(ASSET_SOURCES)
	$(CC) -c $(CFLAGS) -o $@ $< $(LDFLAGS)
# }

# final binary
$(TARGET): $(OBJ_FILES)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
