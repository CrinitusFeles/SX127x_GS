

from sx127x_gs.radio_controller import RadioController

def on_receive(data):
    print(data)
    freq_error = lora.get_lora_fei(lora.get_lora_bw_khz())
    print(freq_error)
    lora.add_freq(freq_error)
    # print(lora.get_lora_fei(lora.bw))

if __name__ == '__main__':
    lora: RadioController = RadioController(interface='Serial')
    lora.received.connect(on_receive)
    if lora.connect(port_or_ip='COM5'):  # 192.168.0.5
        print(lora.get_lora_bandwidth().value)

        lora.user_cli()