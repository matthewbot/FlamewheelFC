#include "drivers/xbee.h"
#include "drivers/util.h"
#include "drivers/stm32f4xx_exts.h"
#include "kernel/sync.h"
#include <string.h>

// pin constants
static constexpr int PIN_CTS = 0;
static constexpr int PIN_RTS = 1;
static constexpr int PIN_TX = 2;
static constexpr int PIN_RX = 3;
static constexpr int AF_USART2 = 7;

// DMA constants
static constexpr int tx_stream_num = 6;
static constexpr DMA_Stream_TypeDef *tx_stream = DMA1_Stream6;
static constexpr int rx_stream_num = 5;
static constexpr DMA_Stream_TypeDef *rx_stream = DMA1_Stream5;

// UART constants
static constexpr USART_TypeDef *usart = USART2;
static constexpr uint32_t brr = UART_BRR(42e6, 115200);

// XBee constants
static constexpr int API_TX_16 = 0x01;
static constexpr int API_AT_COMMAND = 0x08;

static constexpr int API_RX_16 = 0x81;
static constexpr int API_AT_RESPONSE = 0x88;
static constexpr int API_TX_STATUS = 0x89;

// helper functions
static uint8_t *begin_packet(uint16_t len);
static uint16_t end_packet(uint8_t *begin, uint8_t *end);
static void send_packet(size_t len);

// buffers
static uint8_t tx_buf[128];
static uint8_t rx_buf[128];
static uint32_t rx_chkfails;

// rx state machine
enum class RXState { START, LEN_H, LEN_L };
static RXState rx_state;
static uint16_t rx_len;
static Signal rx_signal;

// received message
static bool msg_received;
static uint8_t msg_rssi;
static uint16_t msg_addr;
static uint16_t msg_len;
static uint8_t msg_data[100];

// received command response
static bool cmdresp_received;
static XBeeCommandResponse cmdresp;
static uint16_t cmdresp_len;
static uint8_t cmdresp_data[32];

// recevied tx response
static bool sendresp_received;
static XBeeSendResponse sendresp;

void xbee_init() {
    // configure clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    __DMB();

    // configure receive stream on DMA
    rx_stream->PAR = (uint32_t)&usart->DR;
    rx_stream->M0AR = (uint32_t)rx_buf;
    rx_stream->CR = (4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
    util_enable_irq(DMA1_Stream5_IRQn, IRQ_PRI_KERNEL);

    // configure transmit stream on DMA
    tx_stream->PAR = (uint32_t)&usart->DR;
    tx_stream->M0AR = (uint32_t)tx_buf;
    tx_stream->CR = (4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_MEM2PER;

    // configure USAR
    usart->BRR = brr;
    usart->CR3 = USART_CR3_CTSE | USART_CR3_RTSE | USART_CR3_DMAT;
    usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    util_enable_irq(USART2_IRQn, IRQ_PRI_HIGH);

    // configure GPIOs
    GPIOA->AFR[0] |= AFRL(PIN_CTS, AF_USART2) | AFRL(PIN_RTS, AF_USART2) | AFRL(PIN_TX, AF_USART2) | AFRL(PIN_RX, AF_USART2);
    GPIOA->MODER |= MODER_AF(PIN_CTS) | MODER_AF(PIN_RTS) | MODER_AF(PIN_TX) | MODER_AF(PIN_RX);
}

XBeeCommandResponse xbee_send_at_command(const char *data, size_t data_len, char *response, size_t response_len) {
    uint8_t *begin = begin_packet(data_len + 2);
    uint8_t *end = begin;
    *end++ = API_AT_COMMAND;
    *end++ = 0;
    for (int i=0; i<data_len; i++)
        *end++ = data[i];
    uint16_t packet_len = end_packet(begin, end);

    cmdresp_received = false;
    send_packet(packet_len);
    while (!cmdresp_received)
        rx_signal.wait();

    if (response) {
        KernelCriticalSection crit;
        size_t len = cmdresp_len < response_len ? cmdresp_len : response_len;
        memcpy(response, cmdresp_data, len);
    }
}

XBeeSendResponse xbee_send(uint16_t addr, const char *data, size_t data_len) {
    uint8_t *begin = begin_packet(data_len + 5);
    uint8_t *end = begin;
    *end++ = API_TX_16;
    *end++ = 1;
    *end++ = addr >> 8;
    *end++ = addr & 0xFF;
    *end++ = 0;

    for (int i=0; i<data_len; i++)
        *end++ = data[i];
    uint16_t packet_len = end_packet(begin, end);

    sendresp_received = false;
    send_packet(packet_len);
    while (!sendresp_received)
        rx_signal.wait();

    return sendresp;
}

int xbee_receive(char *data, XBeeReceiveHeader &header) {
    while (!msg_received)
        rx_signal.wait();

    KernelCriticalSection crit;
    msg_received = false;
    memcpy(data, msg_data, msg_len);
    header.addr = msg_addr;
    header.rssi = msg_rssi;
    return msg_len;
}

static uint8_t *begin_packet(uint16_t len) {
    uint8_t *pos = tx_buf;
    *pos++ = 0x7E;
    *pos++ = len >> 8;
    *pos++ = len & 0xFF;
    return pos;
}

static uint16_t end_packet(uint8_t *begin, uint8_t *end) {
    uint16_t len = end - begin;
    uint8_t sum = 0;
    while (begin != end)
        sum += *begin++;
    *end++ = 0xFF - sum;
    return len;
}

static void send_packet(size_t len) {
    DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6; // clear status bits
    tx_stream->NDTR = len+4;
    tx_stream->CR |= DMA_SxCR_EN;
}

static void receive_packet() {
    uint8_t sum = 0;
    for (int i=0; i<rx_len; i++)
        sum += rx_buf[i];

    if (sum != 0xFF) {
        rx_chkfails++;
        return;
    }

    switch (rx_buf[0]) {
    case API_RX_16:
        msg_addr = (rx_buf[1] << 8) | rx_buf[2];
        msg_rssi = rx_buf[3];
        msg_len = rx_len - 6;
        memcpy(msg_data, &rx_buf[5], msg_len);
        msg_received = true;
        break;

    case API_AT_RESPONSE:
        cmdresp = (XBeeCommandResponse)rx_buf[4];
        cmdresp_len = rx_len - 6;
        memcpy(cmdresp_data, &rx_buf[7], cmdresp_len);
        cmdresp_received = true;
        break;

    case API_TX_STATUS:
        sendresp = (XBeeSendResponse)rx_buf[2];
        sendresp_received = true;
        break;
    }

    rx_signal.notify_all();
}

extern "C" void irq_usart2() {
    uint8_t byte = usart->DR;

    switch (rx_state) {
    case RXState::START:
        if (byte == 0x7E)
            rx_state = RXState::LEN_H;
        break;

    case RXState::LEN_H:
        rx_len = byte << 8;
        rx_state = RXState::LEN_L;
        break;

    case RXState::LEN_L:
        rx_len |= byte;
        rx_len += 1; // include checksum
        rx_state = RXState::START;

        usart->CR1 &= ~USART_CR1_RXNEIE;
        usart->CR3 |= USART_CR3_DMAR;

        rx_stream->NDTR = rx_len;
        rx_stream->CR |= DMA_SxCR_EN;
        break;
    }
}

extern "C" void irq_dma1_stream5() {
    receive_packet();

    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5; // clear status bits
    usart->CR3 &= ~USART_CR3_DMAR;
    usart->CR1 |= USART_CR1_RXNEIE;
}
