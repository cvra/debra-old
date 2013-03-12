#include "commandline.h"

#include <parse.h>
#include <rdline.h>
#include <string.h>

static void write_char(char c) {
    putchar(c);
}

static struct rdline rdl;
static char prompt[] = "board>";

extern parse_ctx_t commands[];

/* Cette fonction est appellee quand l'utilisateur appuie
 * sur enter. */
void validate_buffer(const char *buf, __attribute__((unused)) int size) {
	int ret;

	ret = parse(commands, buf);
	if (ret == PARSE_AMBIGUOUS)
		printf("Ambiguous command\r\n");
	else if (ret == PARSE_NOMATCH)
		printf("Command not found\r\n");
	else if (ret == PARSE_BAD_ARGS)
		printf("Bad arguments\r\n");
}

/** Cette fonction est appelee a chaque fois que l'utilisateur
 * appuie sur tab. */

static int complete_buffer(const char *buf, char *dst_buf, int dst_size, int *state) {
    return complete(commands, buf, state, dst_buf, dst_size);
}

void commandline_input_char(char c) {
    int same;
    int ret;
    const char *buffer, *history;
    ret = rdline_char_in(&rdl, c);
    if (ret != 2 && ret != 0) {
        buffer = rdline_get_buffer(&rdl);
        history = rdline_get_history_item(&rdl, 0);
        if (history) {
            same = !memcmp(buffer, history, strlen(history)) &&
            buffer[strlen(history)] == '\n';
        }
        else
            same = 0;
        if (strlen(buffer) > 1 && !same)
            rdline_add_history(&rdl, buffer);
        rdline_newline(&rdl, prompt);
    }
}

void commandline_init(void) {
    rdline_init(&rdl, write_char, validate_buffer, complete_buffer);
    rdline_newline(&rdl, prompt);

}
