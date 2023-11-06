


import time
from sx127x_gs.sx127x_driver import SX127x_Driver
from sx127x_gs.sx127x_registers_and_params import (SX127x_DcFree, SX127x_Modulation,
                                                                       SX127x_ReastartRxMode)


def init_fsk(ax25_mode: bool = False):
    lora.interface.reset()
    time.sleep(0.1)

    lora.set_modulation(SX127x_Modulation.FSK)
    lora.set_frequency(436_500_000)
    lora.set_tx_power(-3)
    lora.set_pa_select(True)

    lora.set_fsk_bitrate(9600)
    lora.set_fsk_deviation(4819)
    lora.set_fsk_sync_mode(True)
    lora.set_fsk_payload_length(256)
    lora.set_fsk_preamble_length(3)
    lora.set_fsk_sync_size(5)
    if ax25_mode:
        lora.set_fsK_packet_format(False)
        lora.set_fsk_dc_free_mode(SX127x_DcFree.OFF)
        lora.set_fsk_crc(False)
        lora.set_fsk_sync_value(bytes([0xFE, 0xFB, 0x91, 0xC5, 0xD5, 0xBE]))
        lora.set_fsk_restart_rx_mode(SX127x_ReastartRxMode.WAIT_PLL)
    else:
        lora.set_fsK_packet_format(True)
        lora.set_fsk_dc_free_mode(SX127x_DcFree.WHITENING)
        lora.set_fsk_crc(True)
        lora.set_fsk_sync_value(b'NSUNET')
        lora.set_fsk_restart_rx_mode(SX127x_ReastartRxMode.NO_WAIT_PLL)

    lora.set_standby_mode()
    time.sleep(0.1)

def fsk_receiving():
    lora.set_rx_continuous_mode()
    lora.interface.write_fsk_read_start()
    time.sleep(0.1)
    print(lora.get_operation_mode())
    try:
        while True:
            print(lora.get_fsk_isr_list())
            if 'PAYLOAD_READY' in lora.get_fsk_isr_list():
                freq_error = lora.get_fsk_fei()
                print(f'{freq_error=}')
                lora.add_freq(freq_error)
                # data_len = lora.interface.read(SX127x_Registers.FIFO.value)
                # print(data_len)
                # print(bytes(lora.interface.read_several(SX127x_Registers.FIFO.value, data_len)))
                time.sleep(1)
                print(lora.interface.write_fsk_read())
            time.sleep(1)
    except KeyboardInterrupt:
        lora.reset_irq_flags()
        print(lora.get_fsk_isr_list())


if __name__ == '__main__':
    lora: SX127x_Driver = SX127x_Driver('Serial')
    lora.connect('COM5')
    init_fsk()
    lora.add_freq_ppm(-4.5)
    fsk_receiving()