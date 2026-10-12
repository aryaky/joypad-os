// uart_cli.h - Accepts the same command set as USB CDC on the debug UART.
//
// Boards wired up with only TX/RX (and no USB host on the CDC port) had no way
// to send configuration. This reads one line at a time from the debug UART and
// hands it to the CDC command dispatcher, so both transports answer the same
// commands and there is only one command table to keep honest.

#ifndef UART_CLI_H
#define UART_CLI_H

#ifdef __cplusplus
extern "C" {
#endif

// Call repeatedly from the main loop. Cheap when nothing has arrived.
void uart_cli_task(void);

#ifdef __cplusplus
}
#endif

#endif // UART_CLI_H
