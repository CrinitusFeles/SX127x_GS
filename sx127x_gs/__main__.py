

import time
import sys
from sx127x_gs.radio_controller import RadioController


if __name__ == '__main__':
    lora: RadioController = RadioController(interface='Serial')
    if lora.connect(port_or_ip=sys.argv[1]):  # 192.168.0.5
        time.sleep(0.2)
        lora.user_cli()