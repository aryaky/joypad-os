// uart_cli.c - see uart_cli.h

#include "uart_cli.h"

#include "hardware/uart.h"
#include "uart.h"
#include "usb/usbd/cdc/cdc_commands.h"

#include <string.h>

#define UART_CLI_MAX_LINE 256

// Accept either a bare newline or a carriage return as the end of a line, so
// the same board works with terminal programs that send one or the other.
static char line[UART_CLI_MAX_LINE];
static size_t line_len = 0;

static void uart_cli_reset(void)
{
    line_len = 0;
    line[0] = '\0';
}

void uart_cli_task(void)
{
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);

        if (c == '\n' || c == '\r') {
            if (line_len == 0) {
                continue;  // ignore bare line endings between commands
            }
            line[line_len] = '\0';
            cdc_commands_process_uart_line(line);
            uart_cli_reset();
            continue;
        }

        if (line_len + 1 >= UART_CLI_MAX_LINE) {
            // Over-long line: drop it rather than feeding a truncated command,
            // which would otherwise be answered as if it were valid.
            uart_cli_reset();
            continue;
        }

        line[line_len++] = c;
    }
}
