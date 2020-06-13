OUTPUT := bin/prog_otter
SOURCES := src/prog_otter.c

.PHONY: all
all: $(OUTPUT) ;

$(OUTPUT): $(SOURCES)
	$(CC) $^ -lreadline -o $@

.PHONY: clean
clean:
	$(RM) bin/prog_otter
