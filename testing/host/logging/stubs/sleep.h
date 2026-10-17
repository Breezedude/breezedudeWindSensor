#pragma once
// Stub for the host test: logging.cpp only needs the one function from
// sleep.h that log_ser_begin() calls after re-opening the debug UART.
void configure_debug_uart_standby(bool enable);
