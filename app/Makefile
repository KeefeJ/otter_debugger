OUTPUT := bin/dbg_otter
SOURCES := src/dbg_otter.c

.PHONY: all
all: $(OUTPUT) ;

$(OUTPUT): $(SOURCES)
	$(CC) $^ -lreadline -o $@

.PHONY: clean
clean:
	$(RM) $(OUTPUT)
