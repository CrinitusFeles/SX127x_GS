from __future__ import annotations
from datetime import datetime, UTC
from threading import Thread, Lock
import time
from queue import Queue, Empty

from ast import literal_eval
from typing import Callable, Iterable
from loguru import logger
from event.event import Event
from sx127x_gs.driver import SX127x_Driver
from sx127x_gs.models import LoRaRxPacket, LoRaTxPacket, RadioModel
from sx127x_gs.registers_and_params import (SX127x_BW, SX127x_CR,
                                            SX127x_HeaderMode, SX127x_Mode,
                                            SX127x_Modulation)


def sleep(timeout: float):
    counter = 0
    while counter < timeout:
        time.sleep(0.01)
        counter += 0.01

class RadioController(SX127x_Driver):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)  # super(LoRa_Controller, self).__init__(**kwargs)
        self.modulation: SX127x_Modulation = kwargs.get('modulation', SX127x_Modulation.LORA)
        self.coding_rate: SX127x_CR = kwargs.get('ecr', self.cr.CR5)  # error coding rate
        self.bandwidth: SX127x_BW = kwargs.get('bw', self.bw.BW250)  # bandwidth  BW250
        self.spread_factor: int = kwargs.get('sf', 10)  # spreading factor  SF10
        self.frequency: int = kwargs.get('frequency', 436_000_000)   # 436700000
        self.crc_mode: bool = kwargs.get('crc_mode', True)  # check crc
        self.tx_power: int = kwargs.get('tx_power', 10)  # dBm
        self.sync_word: int = kwargs.get('sync_word', 0x12)
        self.preamble_length: int = kwargs.get('preamble_length', 8)
        self.auto_gain_control: bool = kwargs.get('agc', True)  # auto gain control
        self.payload_length: int = kwargs.get('payload_size', 10)  # for implicit mode
        self.low_noize_amplifier: int = kwargs.get('low_noize_amplifier', 5)  # 1 - min; 6 - max
        self.lna_boost: bool = kwargs.get('lna_boost', False)  # 150% LNA current
        self.header_mode: SX127x_HeaderMode = kwargs.get('header_mode', SX127x_HeaderMode.EXPLICIT) # fixed payload size
        self.low_data_rate_optimize: bool = kwargs.get('low_data_rate_optimize', True)
        self.only_tx: bool = kwargs.get('only_tx', False)
        self.label: str = kwargs.get('label', '')
        self.transmited: Event = Event(LoRaTxPacket)
        self.received: Event = Event(LoRaRxPacket)
        self.received_raw: Event = Event(bytes)
        self.tx_timeout: Event = Event(str)
        self.on_rx_timeout: Event = Event(str)

        self.__rx_thread = Thread(name=f'{self.label}_rx_thread',
                                  target=self.rx_routine,
                                  daemon=True)
        self.__stop_rx_routine_flag: bool = False
        self._rx_queue: Queue[LoRaRxPacket] = Queue()
        self._tx_buffer: list[LoRaTxPacket] = []
        self._rx_buffer: list[LoRaRxPacket] = []
        self.__lock = Lock()
        self._last_caller: str = ''
        self._repeating_flag: bool = True

    def stop_repeating(self) -> None:
        self._repeating_flag = False

    def clear_subscribers(self) -> None:
        self.received.subscribers[:] = self.received.subscribers[:2]
        self.transmited.subscribers[:] = self.transmited.subscribers[:2]
        self.on_rx_timeout.subscribers.clear()
        self.tx_timeout.subscribers.clear()
        self._last_caller: str = ''

    def init(self) -> None:
        self.stop_rx_thread()
        self.interface.reset()
        time.sleep(0.1)
        self.set_modulation(self.modulation)
        self.set_lora_header_mode(self.header_mode)
        if self.header_mode == SX127x_HeaderMode.IMPLICIT:
            self.set_lora_payload_length(self.payload_length)
        self.set_lora_coding_rate(self.coding_rate)
        self.set_lora_bandwidth(self.bandwidth)
        self.set_lora_sf(self.spread_factor)
        self.set_lora_crc_mode(self.crc_mode)
        self.set_tx_power(self.tx_power)
        self.set_lora_sync_word(self.sync_word)
        self.set_lora_preamble_length(self.preamble_length)
        self.set_lora_auto_gain_control(self.auto_gain_control)
        # if not self.auto_gain_control:
        self.set_low_noize_amplifier(self.low_noize_amplifier, self.lna_boost)
        self.set_lora_rx_tx_fifo_base_addr(0, 0)
        self.set_frequency(self.frequency)
        self.set_low_data_rate_optimize(self.low_data_rate_optimize)
        if not self.only_tx:
            self.to_receive_mode()
        self.start_rx_thread()

    def to_model(self) -> RadioModel:
        if self.interface.connection_status:
            op_mode: str = self.get_operation_mode().name
        else:
            op_mode = 'SLEEP'
        return RadioModel(mode=self.modulation.name,
                          frequency=self.frequency,
                          spreading_factor=self.spread_factor,
                          bandwidth=self.bandwidth.name,
                          check_crc=self.crc_mode,
                          sync_word=self.sync_word,
                          coding_rate=self.coding_rate.name,
                          tx_power=self.tx_power,
                          lna_boost=self.lna_boost,
                          lna_gain=self.low_noize_amplifier,
                          header_mode=self.header_mode.name,
                          autogain_control=self.auto_gain_control,
                          ldro=self.low_data_rate_optimize,
                          op_mode=op_mode)

    def read_config(self) -> RadioModel:
        self.stop_rx_thread()
        modulation = self.get_modulation()
        bw = self.get_lora_bandwidth()
        op_mode = self.get_operation_mode()
        cr = self.get_lora_coding_rate()
        header_mode = self.get_lora_header_mode()
        model = RadioModel(mode=modulation.name if modulation else '',
                          op_mode=op_mode.name if op_mode else '',
                          frequency=self.get_freq(),
                          spreading_factor=self.get_lora_sf(),
                          bandwidth=bw.name if bw else '',
                          check_crc=self.get_lora_crc_mode(),
                          sync_word=self.get_lora_sync_word(),
                          coding_rate=cr.name if cr else '',
                          tx_power=self.get_tx_power_dbm(),
                          autogain_control=self.get_lora_auto_gain_control(),
                          lna_boost=self.get_lna_boost(),
                          lna_gain=self.get_lna_gain(),
                          header_mode=header_mode.name if header_mode else '',
                          ldro=self.get_low_data_rate_optimize())
        self.start_rx_thread()
        return model

    def start_rx_thread(self) -> None:
        if not self.__rx_thread.is_alive() and not self.only_tx:
            logger.debug('Start Rx thread')
            self.__stop_rx_routine_flag = False
            label: str = f'radio_{self.label}_rx_thread'
            self.__rx_thread = Thread(name=label,
                                        target=self.rx_routine,
                                        daemon=True)
            self.__rx_thread.start()

    def stop_rx_thread(self) -> None:
        if self.__rx_thread.is_alive():
            self.__stop_rx_routine_flag = True
            self.__rx_thread.join(timeout=0.8)

    def connect(self, port_or_ip: str) -> bool:
        if super().connect(port_or_ip):
            time.sleep(0.1)
            logger.success(f'Radio {self.label} connected.\nStart initialization...')
            self.init()
            logger.success(f'Radio {self.label} inited.')
            return True
        logger.warning(f'Radio {self.label} is not connected!')
        return False
            # raise Exception("Can't connect to radio.")

    def disconnect(self) -> bool:
        self.stop_rx_thread()
        return super().disconnect()

    def _send_chunks(self, data: bytes, chunk_size: int) -> None:
        chunks: list[bytes] = [data[i:i + chunk_size]
                               for i in range(0, len(data), chunk_size)]
        logger.debug(f'{self.label} big parcel: {len(data)=}')
        is_implicit: bool = (self.header_mode == SX127x_HeaderMode.IMPLICIT)
        for chunk in chunks:
            tx_chunk: LoRaTxPacket = self.calculate_packet(chunk)
            self._tx_buffer.append(tx_chunk)
            logger.debug(tx_chunk)
            self.write_fifo(chunk, is_implicit)
            self.interface.run_tx_then_rx_cont()
            time.sleep((tx_chunk.Tpkt + 10) / 1000)

    def send_single(self, data: bytes, caller_name: str = '') -> LoRaTxPacket:
        self.stop_rx_thread()
        buffer_size: int = 255
        tx_pkt: LoRaTxPacket = self.calculate_packet(data)
        tx_pkt.caller = caller_name
        self._last_caller = caller_name
        logger.debug(f'{self.label}{tx_pkt}')
        if len(data) > buffer_size:
            self._send_chunks(data, buffer_size)
        else:
            self._tx_buffer.append(tx_pkt)
            is_implicit: bool = (self.header_mode == SX127x_HeaderMode.IMPLICIT)
            self.write_fifo(data, is_implicit)
            self.interface.run_tx_then_rx_cont()
            time.sleep((tx_pkt.Tpkt) / 1000)
            self.reset_irq_flags()

        with self.__lock:
            self.transmited.emit(tx_pkt)

        self.start_rx_thread()
        return tx_pkt

    def to_receive_mode(self) -> None:
        mode: SX127x_Mode = self.get_operation_mode()
        if mode != self.mode.RXCONT:
            if mode != self.mode.STDBY:
                self.set_standby_mode()
            self.set_rx_continuous_mode()

    def calculate_freq_error(self) -> int:
        # if self.sat_path:
        #     light_speed = 299_792_458  # m/s
        #     range_rate = int(self.sat_path.find_nearest(self.sat_path.dist_rate, datetime.now().astimezone(utc)) * 1000)
        #     return self.frequency - int((1 + range_rate / light_speed) * self.frequency)
        return 0

    def calculate_packet(self, packet: bytes,
                         force_optimization=True) -> LoRaTxPacket:
        sf: int = self.spread_factor
        _str_bw: str = self.bandwidth.name.replace('BW', '').replace('_', '.')
        bw: int | float = literal_eval(_str_bw)
        cr: int = self.coding_rate.value >> 1
        if self.header_mode == SX127x_HeaderMode.IMPLICIT:
            payload_size = self.payload_length
        else:
            payload_size: int = len(packet)
        t_sym: float = 2 ** sf / bw  # ms
        optimization_flag: bool = True if force_optimization else t_sym > 16
        preamble_time: float = (self.preamble_length + 4.25) * t_sym
        _tmp_1: int = 8 * payload_size - 4 * sf + 28
        _tmp_2: int = 16 * self.crc_mode - 20 * self.header_mode.value
        tmp_poly: int = max((_tmp_1 + _tmp_2), 0)
        _devider = (4 * (sf - 2 * optimization_flag))
        payload_symbol_nb: float = 8 + (tmp_poly / _devider) * (4 + cr)
        payload_time: float = payload_symbol_nb * t_sym
        packet_time: float = payload_time + preamble_time
        timestamp: datetime = datetime.now().astimezone()

        return LoRaTxPacket(timestamp.isoformat(' ', 'seconds'),
                            packet.hex(' ').upper(), len(packet),
                            self.calculate_freq_error(), self.frequency,
                            packet_time, optimization_flag)

    def get_rssi_packet(self) -> int:
        raw_val: int = self.interface.read(self.reg.LORA_PKT_RSSI_VALUE.value)
        return raw_val - (164 if self.frequency < 0x779E6 else 157)

    def get_rssi_value(self) -> int:
        raw_val: int = self.interface.read(self.reg.LORA_RSSI_VALUE.value)
        return raw_val - (164 if self.frequency < 0x779E6 else 157)

    def get_snr(self) -> int:
        return self.interface.read(self.reg.LORA_PKT_SNR_VALUE.value) // 4

    def get_snr_and_rssi(self) -> tuple[int, int]:
        data: list[int] = self.interface.read_several(self.reg.LORA_PKT_SNR_VALUE.value, 2)
        if len(data) >= 2:
            snr, rssi = data[0], data[1]
            return snr // 4, rssi - (164 if self.frequency < 0x779E6 else 157)
        logger.warning(f'{self.label} get_snr_and_rssi ERROR!')
        return 0, 0

    def send_repeat(self, data: bytes | Callable,
                    period_sec: float,
                    untill_answer: bool = True,
                    max_retries: int = 50,
                    answer_handler: Callable[[LoRaRxPacket, Iterable], bool] | None = None,
                    handler_args: Iterable = (),
                    caller_name: str = '') -> LoRaRxPacket | None:
        last_rx_packet: LoRaRxPacket | None = None
        # retries: int = max_retries if max_retries > 0 else 99999


        if self.only_tx:
            while max_retries:
                bdata: bytes = data() if isinstance(data, Callable) else data
                tx_packet: LoRaTxPacket = self.send_single(bdata, caller_name)
                timeout: float = period_sec - tx_packet.Tpkt / 1000
                sleep(timeout)
                max_retries -= 1
            return None

        while self._rx_queue.qsize() > 0:
            print(self._rx_queue.get_nowait())

        while max_retries and self._repeating_flag:
            bdata: bytes = data() if isinstance(data, Callable) else data
            tx_packet: LoRaTxPacket = self.send_single(bdata, caller_name)
            timeout: float = period_sec - tx_packet.Tpkt / 1000
            try:
                rx_packet: LoRaRxPacket = self._rx_queue.get(timeout=timeout)
                last_rx_packet = rx_packet
                if not rx_packet.is_crc_error and untill_answer:
                    if answer_handler:
                        if answer_handler(rx_packet, *handler_args):
                            break
                    else:
                        break
            except Empty:
                logger.debug(f'{self.label} rx_timeout')
            max_retries -= 1
        self._repeating_flag = True
        return last_rx_packet


    def check_rx_input(self) -> LoRaRxPacket | None:
        if not self.get_rx_done_flag():
            return None

        curr_addr: int = self.interface.read(self.reg.LORA_FIFO_RX_CURRENT_ADDR.value)
        # TODO: remember previous address to minimize tcp packet (do not need to set fifo address ptr every time)
        self.interface.write(self.reg.LORA_FIFO_ADDR_PTR.value, [curr_addr])
        freq_error: int = self.calculate_freq_error()
        if self.header_mode == SX127x_HeaderMode.IMPLICIT:
            data: list[int] = self.interface.read_several(self.reg.FIFO.value,
                                                          self.payload_length)
        else:
            rx_amount: int = self.interface.read(self.reg.LORA_RX_NB_BYTES.value)
            data = self.interface.read_several(self.reg.FIFO.value, rx_amount)
        crc: bool = self.get_crc_flag()
        self.reset_irq_flags()
        fei: int = self.get_lora_fei(self.get_lora_bw_khz())
        timestamp: str = datetime.now().astimezone(UTC).isoformat(' ', 'seconds')
        snr, rssi = self.get_snr_and_rssi()
        return LoRaRxPacket(timestamp=timestamp,
                            data=' '.join(f'{val:02X}' for val in data),
                            data_len=len(data),
                            freq_error_hz=freq_error,
                            frequency=self.frequency,
                            snr=snr,
                            rssi_pkt=rssi,
                            is_crc_error=crc,
                            fei=fei,
                            caller=self._last_caller)

    # def dump_memory(self) -> SX127x_Registers:
    #     dump_mem: list[int] = self.get_all_registers()
    #     mem = {k: dump_mem[v - 1] for k, v in self.reg.items()}
    #     return SX127x_Registers(mem)

    def clear_rx_buffer(self) -> None:
        self._rx_queue.queue.clear()
        self._rx_buffer.clear()

    def clear_tx_buffer(self) -> None:
        self._tx_buffer.clear()

    def clear_buffers(self) -> None:
        self.clear_rx_buffer()
        self.clear_tx_buffer()

    def clear(self) -> None:
        self.clear_buffers()
        self.sat_path = None
        self.clear_subscribers()

    def get_tx_buffer(self) -> list[LoRaTxPacket]:
        return self._tx_buffer

    def get_rx_buffer(self) -> list[LoRaRxPacket]:
        return self._rx_buffer

    def rx_routine(self) -> None:
        while not self.__stop_rx_routine_flag:
            pkt: LoRaRxPacket | None = self.check_rx_input()
            if pkt is not None:
                if len(pkt.data) > 0:
                    logger.debug(pkt)
                    self._rx_queue.put(pkt)
                    self._rx_buffer.append(pkt)
                with self.__lock:
                    self.received.emit(pkt)
                    self.received_raw.emit(pkt.to_bytes())
            time.sleep(0.01)

    def set_frequency(self, new_freq_hz: int) -> None:
        super().set_frequency(new_freq_hz)
        self.frequency = new_freq_hz

    def user_cli(self) -> None:
        try:
            while True:
                data: str = input('> ')
                if not data:
                    continue
                try:
                    list_data: list = literal_eval(data)
                    bdata = bytes(list_data)
                    self.send_single(bdata)
                except (SyntaxError, ValueError):
                    self.send_single(data.encode())

        except KeyboardInterrupt:
            self.disconnect()
            logger.debug('Shutdown radio driver')


if __name__ == '__main__':
    lora: RadioController = RadioController(interface='Serial', tx_power=2)
    if lora.connect(port_or_ip='COM7'):  # 192.168.0.5
        print(lora.read_config())
        lora.user_cli()
        # lora.user_cli()
        # is_868 = True
        # try:
        #     while True:

        #         if is_868:
        #             lora.set_frequency(868_000_000)
        #         else:
        #             lora.set_frequency(915_000_000)
        #         time.sleep(0.5)
        #         lora.init()
        #         time.sleep(1)

        #         lora.send_single([i for i in range(100)])
        #         time.sleep(2)
        #         is_868 = not is_868
        # except KeyboardInterrupt:
        #     pass



# FSK mode:
# RegBitrate(0x02, 0x03): x = 9600;
# RegPreambleLsb(0x26) = 3;
# RegSyncValue(0x28) = NSUNET\0\0;
# RegSyncConfig(0x27) AutoRestartRxMode = 01 -> On, without waiting for the PLL to re-lock
# RegSyncConfig(0x27) SyncSize = 5 (sizeof(NSUNET) - 1)
# RegPacketConfig1(0x30) DcFree = Whitening