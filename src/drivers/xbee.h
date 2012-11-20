#ifndef FC_DRIVER_XBEE_H
#define FC_DRIVER_XBEE_H

#include <stdlib.h>
#include <stdint.h>

void xbee_init();

enum class XBeeCommandResponse { OK, ERROR, INVCMD, INVPAR };
XBeeCommandResponse xbee_send_at_command(const char *data, size_t data_len, char *response, size_t response_len);

enum class XBeeSendResponse { SUCCESS, NOACK, CCAFAIL, PURGED };
XBeeSendResponse xbee_send(uint16_t addr, const char *data, size_t data_len);

struct XBeeReceiveHeader {
    uint16_t addr;
    uint8_t rssi;
};

bool xbee_receive_avail();
int xbee_receive(char *data, XBeeReceiveHeader &header);

#endif
