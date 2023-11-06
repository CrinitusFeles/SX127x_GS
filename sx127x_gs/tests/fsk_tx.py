

import time
from sx127x_gs.radio_controller import RadioController
from sx127x_gs.sx127x_registers_and_params import (FromStart, FromTransmit, IdleMode,
                                                                       LowPowerSelection, SX127x_DcFree,
                                                                       SX127x_Modulation, SX127x_ReastartRxMode)
def on_receive(data):
    print(data)
    freq_error = lora.get_lora_fei(lora.get_lora_bw_khz())
    print(freq_error)
    lora.add_freq(freq_error)
    init_fsk(True)
    fsk_receiving()

def init_fsk(ax25_mode: bool = False):
    lora.interface.reset()
    time.sleep(0.1)

    lora.set_modulation(SX127x_Modulation.FSK)
    lora.set_frequency(436_500_000)
    lora.set_tx_power(-3)
    lora.set_pa_select(True)

    lora.set_fsk_bitrate(9600)
    lora.set_fsk_deviation(5000)
    lora.set_fsk_sync_mode(True)
    lora.set_fsk_payload_length(256)
    lora.set_fsk_preamble_length(3)
    lora.set_fsk_sync_size(6)
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
    _receiving()

def fsk_send():
    lora.interface.write_fsk_fifo(list(bytes.fromhex('0E 0A 06 01 FD 01 01 01 01 00 03 00 00 00 03')))
    # lora.interface.run_tx_then_rx_cont()
    print(lora.fsk_sequencer.read())
    lora.fsk_sequencer.from_start = FromStart.Transmit_on_FIFOLEVEL
    lora.fsk_sequencer.idle_mode = IdleMode.StandbyMode
    lora.fsk_sequencer.from_transmit = FromTransmit.Receive_on_PACKETSENT
    lora.fsk_sequencer.low_power = LowPowerSelection.IdleMode
    lora.fsk_sequencer.upload(True)
    time.sleep(0.1)
    print(lora.get_operation_mode())
    fsk_receiving()

def _receiving():
    try:
        while True:
            print(lora.get_fsk_isr_list())
            if 'PAYLOAD_READY' in lora.get_fsk_isr_list():
                time.sleep(1)
                print(lora.interface.write_fsk_read())
            time.sleep(1)
    except KeyboardInterrupt:
        lora.reset_irq_flags()
        print(lora.get_fsk_isr_list())

if __name__ == '__main__':
    lora: RadioController = RadioController(interface='Serial')
    lora.received.connect(on_receive)
    if lora.connect(port_or_ip='COM17'):  # 192.168.0.5
        # lora.user_cli()
        lora.stop_rx_thread()
        init_fsk(ax25_mode=True)
        lora.add_freq(2519)
        fsk_send()


