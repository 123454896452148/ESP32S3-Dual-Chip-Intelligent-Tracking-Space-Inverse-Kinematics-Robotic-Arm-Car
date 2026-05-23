#ifndef UART_COMM_H
#define UART_COMM_H
class UartComm {
public:
    static UartComm& getInstance();
    void init();
    void send(const char* s);
    int read(char* buf, int len);
};
#endif