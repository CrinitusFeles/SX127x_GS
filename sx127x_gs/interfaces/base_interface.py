from __future__ import annotations
import time
from typing import Any, Callable
from loguru import logger


def check_connection(func: Callable):
    def _wrapper(*args, **kwargs):
        if not args[0].connection_status:
            raise RuntimeError('Radio is not connected')
        return func(*args, **kwargs)
    return _wrapper

def retry(func: Callable[..., bytes], counter: int):
    data = b''
    while counter > 0:
        data = func()
        if data != b'':
            break
        time.sleep(0.15)
        logger.error('read empty bytes')
        counter -= 1
    return data



class BaseInterface:
    _write: Callable[[bytes], int | None]
    _read: Callable[[int], bytes]
    _interface: Any
    connection_status: bool = False

    def connect(self, _: str):
        raise NotImplementedError

    def disconnect(self):
        raise NotImplementedError

    @check_connection
    def read(self, address: int) -> int:
        self._write(bytes([1, address]))
        data: bytes = retry(self._try_read, 5)
        return int.from_bytes(data, "big")

    @check_connection
    def write(self, address: int, data: list[int]) -> int:
        if len(data) == 1:
            self._write(bytes([2, address, data[0]]))
        else:
            self._write(bytes([8, address, len(data), *data]))
        return int.from_bytes(self._try_read(), "big")

    @check_connection
    def run_tx_then_rx_cont(self) -> int:
        self._write(bytes([21]))
        return int.from_bytes(self._try_read(), "big")

    @check_connection
    def run_tx_then_rx_single(self) -> int:
        self._write(bytes([22]))
        return int.from_bytes(self._try_read(), "big")

    @check_connection
    def read_several(self, address: int, amount: int) -> list[int]:
        self._write(bytes([7, address, amount]))
        return list(self._try_read(amount))

    @check_connection
    def reset(self) -> int:
        self._write(bytes([6]))
        return int.from_bytes(self._try_read(), "big")

    @check_connection
    def write_fsk_fifo(self, data: list[int]) -> int:
        send_data = [31, len(data), *data]
        self._write(bytes(send_data))
        return int.from_bytes(self._try_read(), "big")  # 31

    @check_connection
    def write_fsk_read_start(self) -> int:
        """
        Запускает на микроконтроллере процесс перекладывания данных из FIFO приемопередатчика
        в FIFO микроконтроллера. Параллельно небходимо вычитывать статусный регистр, пока
        пакет не придет полностью. После окончания пакета нужно вызвать функцию write_fsk_read.
        Для остановки процесса перекладывания данных можно вызвать функцию write_fsk_read до
        окончания пакета.
        """
        self._write(bytes([32]))
        return int.from_bytes(self._try_read(), "big")

    @check_connection
    def write_fsk_read(self) -> list[int] :
        """
        Первым байтом возвращает количество следующих байт, а потом последующие байты из
        FIFO микроконтроллера.
        """
        self._write(bytes([33]))
        data_len: int = int.from_bytes(self._try_read(), "big")
        if data_len > 0:
            return list(self._try_read(data_len))
        return [data_len]

    def _try_read(self, amount: int = 1) -> bytes:  # type: ignore
        try:
            data: bytes = self._read(amount)
            if data == b'':
                raise RuntimeError('Radio read empty data')
            return data
        except TimeoutError as exc:
            raise TimeoutError(f'Radio reading timeout: {exc}') from exc
        except TypeError as exc:
            logger.error('serial interface can not read None data', exc)