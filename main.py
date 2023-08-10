# MicroPython
# mail: goctaprog@gmail.com
# MIT license
import time

import sys
# Пожалуйста, прочитайте документацию на HSCDTD008A!
# Please read the HSCDTD008A documentation!
from machine import I2C, Pin
import hscdtd008a
from sensor_pack.bus_service import I2cAdapter


def show_state(sen: hscdtd008a.HSCDTD008A):
    sen.refresh_state()
    print(f"active_power_mode: {sensor.active_power_mode}; hi_dynamic_range: {sensor.hi_dynamic_range};")
    print(f"single_meas_mode: {sensor.single_meas_mode}; periodical_meas_mode: {sensor.periodical_meas_mode};")


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) # для примера
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    # Внимание!!!
    # Замените id=1 на id=0, если пользуетесь первым портом I2C !!!
    # Warning!!!
    # Replace id=1 with id=0 if you are using the first I2C port !!!
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) # для примера

    # i2c = I2C(id=1, scl=Pin(27), sda=Pin(26), freq=400_000)  # on Arduino Nano RP2040 Connect and Pico W tested!
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)  # create I2C peripheral at frequency of 400kHz
    adapter = I2cAdapter(i2c)  # адаптер для стандартного доступа к шине

if __name__ == '__main__':
    dly: int = 250
    sensor = hscdtd008a.HSCDTD008A(adapter)
    sensor.setup(active_pwr_mode=True)  # включаю датчик
    print(f"Sensor id: {sensor.get_id()}")
    print(16 * "_")
    show_state(sensor)
    print(16 * "_")
    test_result = sensor.perform_self_test()
    if not test_result:
        print("Sensor not pass self test!!! Broken or invalid sensor mode!!!")
        sys.exit(1)
    print("Sensor self test passed!")
    sensor.start_meas_temp()  # запускаю измерение температуры
    print("Sensor temperature measurement!")
    print(16 * "_")
    show_state(sensor)
    print(16 * "_")
    cnt = 0
    while cnt < 30:
        status = sensor.get_status()
        if status[3]:   # TRDY flag from STAT1 reg
            temp = sensor.get_temperature()
            print(f"Sensor temperature: {temp} ℃")
            sensor.start_meas_temp()
        else:
            print(f"status: {status}")
        time.sleep_ms(dly)
        cnt += 1

    print(16 * "_")
    show_state(sensor)
    print(16 * "_")
    print("Magnetic field measurement!")
    cnt = 0
    sensor.start_measure()
    while cnt < 30:
        status = sensor.get_status()
        if status[0] or status[1]:  # DRDY or DOR flag from STAT1 reg
            # field = [sensor.get_axis(i) for i in range(3)]    # три(!) вызова в цикле
            field = sensor.get_axis(-1)		# все за один(!) вызов
            sensor.start_measure()
            print(f"magnetic field component: X:{field[0]}; Y:{field[1]}; Z:{field[2]}")
        else:
            print(f"status: {status}")
        time.sleep_ms(dly)
        # cnt += 1
