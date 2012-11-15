#ifndef FC_DRIVERS_BOARD_H
#define FC_DRIVERS_BOARD_H

void board_init();

bool board_switch();
void board_set_led(int led, bool on=true);
float board_get_voltage();

#endif
