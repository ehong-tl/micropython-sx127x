from sx127x import SX127x

from examples import LoRaSender
from examples import LoRaReceiver
from examples import LoRaPing

lora_default = {
    "frequency": 869525000,
    "frequency_offset":0,
    "tx_power_level": 14,
    "signal_bandwidth": 125e3,
    "spreading_factor": 9,
    "coding_rate": 5,
    "preamble_length": 8,
    "implicitHeader": False,
    "sync_word": 0x12,
    "enable_CRC": True,
    "invert_IQ": False,
}

lora_pins = {
    'rx_done':26,
    'ss':18,
    'reset':16,
    'sck':5,
    'miso':19,
    'mosi':27,
}

lora = SX127x(pins=lora_pins, parameters=lora_default)

# type = 'sender'
# type = 'receiver'
# type = 'ping_master'
type = 'ping_slave'


if __name__ == '__main__':
    if type == 'sender':
        LoRaSender.send(lora)
    if type == 'receiver':
        LoRaReceiver.receive(lora)
    if type == 'ping_master':
        LoRaPing.ping(lora, master=True)
    if type == 'ping_slave':
        LoRaPing.ping(lora, master=False)
