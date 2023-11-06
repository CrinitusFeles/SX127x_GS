from __future__ import annotations
import serial
from loguru import logger

from sx127x_gs.interfaces.base_interface import BaseInterface


class SerialInterface(BaseInterface):
    def connect(self, port: str) -> bool:
        if self.connection_status:
            return True
        try:
            self._interface: serial.Serial = serial.Serial(port=port, baudrate=500000, timeout=1, write_timeout=1)
            self._read = self._interface.read
            self._write = self._interface.write
            self._interface.dtr = False
            self._interface.write(bytes([6]))
            self._interface.read(1)
            if self._interface.is_open:
                self.connection_status = True
                return True
            raise ConnectionError(f"Can\'t connect to {port}. Probably device is busy")
        except serial.SerialException as err:
            logger.error(err)
            return False

    def disconnect(self) -> bool:
        if self.connection_status:
            self._interface.close()
            self.connection_status = False
            return not self._interface.is_open
        return True

    def _try_read(self, amount: int = 1) -> bytes:  # type: ignore
        try:
            return super()._try_read(amount)
        except serial.SerialException as exc:
            logger.error(exc)

if __name__ == '__main__':
    ser: SerialInterface = SerialInterface()
    print(ser.connect('COM3'))
