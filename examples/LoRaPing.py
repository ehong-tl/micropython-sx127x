from time import ticks_ms


def ping(lora, master):

    last_send_time = 0
    interval = 3000
    counter = 0

    lora.receive()
    while True:
        if master:
            if ticks_ms() - last_send_time > interval:
                last_send_time = ticks_ms()
                message = 'ping ({})'.format(counter)
                print('{} TX: {}'.format(ticks_ms(), message))
                lora.println(message)

                try:
                    payload = lora.listen(1000).decode()
                except AttributeError:
                    print('{} RX: fail'.format(ticks_ms()))
                else:
                    print('{} RX: {}'.format(ticks_ms(), payload))

                print()
                counter += 1
        else:
            if lora.receivedPacket():
                payload = lora.readPayload().decode()
                print('{} RX: {}'.format(ticks_ms(), payload))
                lora.println(payload)
