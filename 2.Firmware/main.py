from sys import stdin, exit
from _thread import start_new_thread
from utime import sleep
import machine
from machine import Pin, PWM
import time
import math
import utime

hndshk=0
status_led_counter = 0
# led_pin = machine.Pin(25, machine.Pin.OUT) #led pin definition for regular pico
led_pin=machine.Pin("LED",machine.Pin.OUT) #led pin definition for pico w

"""------------------------------motor 1 PID tanimlamalari---------------------------"""
# PID değişkenleri ilk basta bunlari sifir olarak tanimlamamiz gerekiyor
last_error1 = 0
integral1 = 0
last_error2 = 0
integral2 = 0
last_error3 = 0
integral3 = 0
last_error4 = 0
integral4 = 0
"""------------------PID çıkışı için minimum ve maksimum değerler---------------------"""
MIN_RPM = 0
MAX_RPM = 3000
encoder_slot = 2 #encoder diskindeki bosluk sayisi
last_time = utime.ticks_ms() # son zaman
last_time_error = utime.ticks_ms() # son zaman
last_time_motor1 = utime.ticks_ms() # son zaman
"""----------------------------encoder1 tanimlamalari---------------------------------"""
encoder_out_pin1 = machine.Pin(14, machine.Pin.IN, machine.Pin.PULL_UP) # out pinini GPIO 14 (GP14) pinine bağla
pulse_count1 = 0 # pulse sayacı
last_state1 = encoder_out_pin1.value() # son durum
rpm1 = 0 # encoder1 hızı
"""----------------------------encoder2 tanimlamalari---------------------------------"""
encoder_out_pin2 = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP) # out pinini GPIO 15 (GP15) pinine bağla
pulse_count2 = 0 # pulse sayacı
last_state2 = encoder_out_pin2.value() # son durum
rpm2 = 0 # encoder2 hızı
"""----------------------------encoder3 tanimlamalari---------------------------------"""
encoder_out_pin3 = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP) # out pinini GPIO 16 (GP16) pinine bağla
pulse_count3 = 0 # pulse sayacı
last_state3 = encoder_out_pin3.value() # son durum
rpm3 = 0 # encoder3 hızı
"""----------------------------encoder4 tanimlamalari---------------------------------"""
encoder_out_pin4 = machine.Pin(17, machine.Pin.IN, machine.Pin.PULL_UP) # out pinini GPIO 17 (GP17) pinine bağla
pulse_count4 = 0 # pulse sayacı
last_state4 = encoder_out_pin4.value() # son durum
rpm4 = 0 # encoder4 hızı

"""---------------------------------Pin atamaları-------------------------------------"""
#--------------motor1---------#
motor1L_EN = PWM(Pin(2))
motor1R_EN = PWM(Pin(3))
motor1LPWM = Pin(4, Pin.OUT)
motor1RPWM = Pin(5, Pin.OUT)
#--------------motor2---------#
motor2L_EN = PWM(Pin(6))
motor2R_EN = PWM(Pin(7))
motor2LPWM = Pin(8, Pin.OUT)
motor2RPWM = Pin(9, Pin.OUT)
#--------------motor3---------#
motor3L_EN = PWM(Pin(18))
motor3R_EN = PWM(Pin(19))
motor3LPWM = Pin(20, Pin.OUT)
motor3RPWM = Pin(21, Pin.OUT)
#--------------motor4---------#
motor4L_EN = PWM(Pin(10))
motor4R_EN = PWM(Pin(11))
motor4LPWM = Pin(12, Pin.OUT)
motor4RPWM = Pin(13, Pin.OUT)

"""------------------------------motor constrains-------------------------------------"""
h = 0.22 # from robot origin to wheel center in meter about y axis
l = 0.24 # from robot origin to wheel center in meter about x axis
r = 0.12 # wheel radius in meter
"""-----------------------------PID parametreleri-------------------------------------"""
default_Kp=1.7
default_Ki=0.001
default_Kd=0

Kp1 = default_Kp
Ki1 = default_Ki
Kd1 = default_Kd
Kp2 = default_Kp
Ki2 = default_Ki
Kd2 = default_Kd
Kp3 = default_Kp
Ki3 = default_Ki
Kd3 = default_Kd
Kp4 = default_Kp
Ki4 = default_Ki
Kd4 = default_Kd
"""----------------------------- hız ve açı değerleri---------------------------------"""
w = 0  #acisal hiz degeri
V = 0  #vektörel hız değeri
A = 90 #robotun x ekseni ile vektörel hiz degerinin yaptigi açı değeri
t_all_motor=2000 #milisecond
t_motor1 = t_all_motor  # belirlediğiniz süre (milisaniye cinsinden)
t_motor2 = t_all_motor
t_motor3 = t_all_motor
t_motor4 = t_all_motor

bufferSize = 1024                 # size of circular buffer to allocate
buffer = [' '] * bufferSize       # circuolar incomming USB serial data buffer (pre fill)
bufferEcho = True                 # USB serial port echo incooming characters (True/False) 
bufferNextIn, bufferNextOut = 0,0 # pointers to next in/out character in circualr buffer
terminateThread = False           # tell 'bufferSTDIN' function to terminate (True/False)
def bufferSTDIN():
    global buffer, bufferSize, bufferEcho, bufferNextIn, terminateThread
    
    while True:                                 # endless loop
        if terminateThread:                     # if requested by main thread ...
            break                               #    ... exit loop
        buffer[bufferNextIn] = stdin.read(1)    # wait for/store next byte from USB serial
        if bufferEcho:                          # if echo is True ...
            print(buffer[bufferNextIn], end='') #    ... output byte to USB serial
        bufferNextIn += 1                       # bump pointer
        if bufferNextIn == bufferSize:          # ... and wrap, if necessary
            bufferNextIn = 0
bufferSTDINthread = start_new_thread(bufferSTDIN, ())
def getByteBuffer():
    global buffer, bufferSize, bufferNextOut, bufferNextIn
    
    if bufferNextOut == bufferNextIn:           # if no unclaimed byte in buffer ...
        return ''                               #    ... return a null string
    n = bufferNextOut                           # save current pointer
    bufferNextOut += 1                          # bump pointer
    if bufferNextOut == bufferSize:             #    ... wrap, if necessary
        bufferNextOut = 0
    return (buffer[n])                          # return byte from buffer
def getLineBuffer():
    global buffer, bufferSize, bufferNextOut, bufferNextIn

    if bufferNextOut == bufferNextIn:           # if no unclaimed byte in buffer ...
        return ''                               #    ... RETURN a null string

    n = bufferNextOut                           # search for a LF in unclaimed bytes
    while n != bufferNextIn:
        if buffer[n] == '\x0a':                 # if a LF found ... 
            break                               #    ... exit loop ('n' pointing to LF)
        n += 1                                  # bump pointer
        if n == bufferSize:                     #    ... wrap, if necessary
            n = 0
    if (n == bufferNextIn):                     # if no LF found ...
            return ''                           #    ... RETURN a null string

    line = ''                                   # LF found in unclaimed bytes at pointer 'n'
    n += 1                                      # bump pointer past LF
    if n == bufferSize:                         #    ... wrap, if necessary
        n = 0

    while bufferNextOut != n:                   # BUILD line to RETURN until LF pointer 'n' hit
        
        if buffer[bufferNextOut] == '\x0d':     # if byte is CR
            bufferNextOut += 1                  #    bump pointer
            if bufferNextOut == bufferSize:     #    ... wrap, if necessary
                bufferNextOut = 0
            continue                            #    ignore (strip) any CR (\x0d) bytes
        
        if buffer[bufferNextOut] == '\x0a':     # if current byte is LF ...
            bufferNextOut += 1                  #    bump pointer
            if bufferNextOut == bufferSize:     #    ... wrap, if necessary
                bufferNextOut = 0
            break                               #    and exit loop, ignoring (i.e. strip) LF byte
        line = line + buffer[bufferNextOut]     # add byte to line
        bufferNextOut += 1                      # bump pointer
        if bufferNextOut == bufferSize:         #    wrap, if necessary
            bufferNextOut = 0
    return line                                 # RETURN unclaimed line of input


"""------------------- Motor functions !!! careful while editing !!!-------------------"""

def motor1_rotate(pid_output1,we_1):
        if we_1 > 0:
            motor1LPWM.value(1) 
            motor1RPWM.value(0) 
        elif we_1 < 0:          
            motor1LPWM.value(0)
            motor1RPWM.value(1)
        else:
            motor1LPWM.value(0)
            motor1RPWM.value(0)
        mapped_we1 = int((pid_output1 *65534) / MAX_RPM)  # map the we_1 value from [-3000, 3000] to [0, 65534]
        motor1L_EN.duty_u16(abs(mapped_we1))
        motor1R_EN.duty_u16(abs(mapped_we1))

def motor2_rotate(pid_output2,we_2): 
        if we_2 > 0:
            motor2LPWM.value(0)
            motor2RPWM.value(1)
        elif we_2 < 0:
            motor2LPWM.value(1)
            motor2RPWM.value(0)
        else:
            motor2LPWM.value(0)
            motor2RPWM.value(0)
        mapped_we2 = int((pid_output2 *65534) / MAX_RPM)  # map the we_1 value from [-3000, 3000] to [0, 65534]
        motor2L_EN.duty_u16(abs(mapped_we2))
        motor2R_EN.duty_u16(abs(mapped_we2))

def motor3_rotate(pid_output3,we_3):
        if we_3 > 0:
            motor3LPWM.value(1)
            motor3RPWM.value(0)
        elif we_3 < 0:
            motor3LPWM.value(0)
            motor3RPWM.value(1)
        else:
            motor3LPWM.value(0)
            motor3RPWM.value(0)
        mapped_we3 = int((pid_output3 *65534) / MAX_RPM)  # map the we_1 value from [-3000, 3000] to [0, 65534]
        motor3L_EN.duty_u16(abs(mapped_we3))
        motor3R_EN.duty_u16(abs(mapped_we3))
        
def motor4_rotate(pid_output4,we_4):
        if we_4 > 0:
            motor4LPWM.value(0)
            motor4RPWM.value(1)
        elif we_4 < 0:
            motor4LPWM.value(1)
            motor4RPWM.value(0)
        else:
            motor4LPWM.value(0)
            motor4RPWM.value(0)
        mapped_we4 = int((pid_output4 *65534) / MAX_RPM)  # map the we_1 value from [-3000, 3000] to [0, 65534]
        motor4L_EN.duty_u16(abs(mapped_we4))
        motor4R_EN.duty_u16(abs(mapped_we4))

def calculate_pid1(error1): #bunun cikisi rpm oluyor
    global integral1, last_error1
    integral1 += error1
    derivative1 = error1 - last_error1
    last_error1 = error1
    pid_output1 = Kp1*error1 + Ki1*integral1 + Kd1*derivative1
    pid_output1 = max(min(pid_output1, MAX_RPM), MIN_RPM)
    return pid_output1
def calculate_pid2(error2): #bunun cikisi rpm oluyor
    global integral2, last_error2
    integral2 += error2
    derivative2 = error2 - last_error2
    last_error2 = error2
    pid_output2 = Kp2*error2 + Ki2*integral2 + Kd2*derivative2
    pid_output2 = max(min(pid_output2, MAX_RPM), MIN_RPM)
    return pid_output2
def calculate_pid3(error3): #bunun cikisi rpm oluyor
    global integral3, last_error3 
    integral3 += error3 # integral3 hesapla
    derivative3 = error3 - last_error3 # Diferansiyel hesapla
    last_error3 = error3
    pid_output3 = Kp3*error3 + Ki3*integral3 + Kd3*derivative3 # PID çıkışını hesapla
    pid_output3 = max(min(pid_output3, MAX_RPM), MIN_RPM) # Minimum ve maksimum değerler arasında sınırla
    return pid_output3
def calculate_pid4(error4): #bunun cikisi rpm oluyor
    global integral4, last_error4
    integral4 += error4
    derivative4 = error4 - last_error4
    last_error4 = error4
    pid_output4 = Kp4*error4 + Ki4*integral4 + Kd4*derivative4
    pid_output4 = max(min(pid_output4, MAX_RPM), MIN_RPM)
    return pid_output4

try:
    inputOption = 'LINE'                    # get input from buffer one BYTE or LINE at a time
    while True:
        try:
            led_pin.toggle() #to see if pico workin
            """-----------------------input lines-------------------------"""
            if inputOption == 'BYTE':           # NON-BLOCKING input one byte at a time
                buffCh = getByteBuffer()        # get a byte if it is available?
                if buffCh:                      # if there is...
                    print (buffCh, end='')      # ...print it out to the USB serial port

            elif inputOption == 'LINE':         # NON-BLOCKING input one line at a time (ending LF)
                buffLine = getLineBuffer()      # get a line if it is available?
                if buffLine:                    # if there is...
                    if "handshake" in buffLine:
                        hndshk = 1
                        print("picoheard")
                    if "V=" in buffLine: # and hndshk == 1:
                        value_index = buffLine.index("V=") + 2  # "V=" değerinin başlangıç indisini bulun
                        V = int(buffLine[value_index:])  # "V=" sonrasındaki değeri tamsayıya dönüştürün
                        print("V value:", V)
                    if "w=" in buffLine: # and hndshk == 1:
                        value_index = buffLine.index("w=") + 2  # "w=" değerinin başlangıç indisini bulun
                        w = int(buffLine[value_index:])  # "w=" sonrasındaki değeri tamsayıya dönüştürün
                        print("w value:", w)
                    if "A=" in buffLine:
                        value_index = buffLine.index("A=") + 2  # "A=" değerinin başlangıç indisini bulun
                        A = int(buffLine[value_index:])  # "A=" sonrasındaki değeri tamsayıya dönüştürün
                        print("A value:", A)
                    if "STOP" in buffLine:
                        V=0
                        w=0
                        print("ROBOT STOPPED!")
                    #print (buffLine)            # ...print it out to the USB serial port
            
            if hndshk == 0:
                V=0
                w=0
            else:
                pass
            """-----------------------MAIN CALCULATON & CONTROL LOOP-------------------------"""
            Vx = V * math.cos(math.radians(A))
            Vy = V * math.sin(math.radians(A))
            # her motor için rpm hesapladığımız matrix çarpımlarının yapıldığı kod
            we_1 = (30 / math.pi) * (1 / r) * (-Vx + Vy + (h + l) * w)
            we_2 = (30 / math.pi) * (1 / r) * (Vx + Vy - (h + l) * w)
            we_4 = (30 / math.pi) * (1 / r) * (-Vx + Vy - (h + l) * w)
            we_3 = (30 / math.pi) * (1 / r) * (Vx + Vy + (h + l) * w)
            #motor_forward(0) # 0-65534 arasindaki pwm frekasinda ileri hareket #max 65534
            """---------------------encoder codes----------------------"""
            current_state1 = encoder_out_pin1.value() # mevcut durum
            if current_state1 != last_state1 and current_state1 == 0:
                pulse_count1 += 1
            last_state1 = current_state1
            current_state2 = encoder_out_pin2.value() # mevcut durum
            if current_state2 != last_state2 and current_state2 == 0:
                pulse_count2 += 1
            last_state2 = current_state2
            current_state3 = encoder_out_pin3.value() # mevcut durum
            if current_state3 != last_state3 and current_state3 == 0:
                pulse_count3 += 1
            last_state3 = current_state3
            current_state4 = encoder_out_pin4.value() # mevcut durum
            if current_state4 != last_state4 and current_state4 == 0:
                pulse_count4 += 1
            last_state4 = current_state4
            current_time = utime.ticks_ms() # mevcut zaman
            time_diff = utime.ticks_diff(current_time, last_time)
            if time_diff >= 500: # 1000ms = 1 saniye
                last_time = current_time
                rpm1 = (pulse_count1 / encoder_slot) * ((60 * time_diff) / 5000) # 12 pulse = 1 tur
                rpm2 = (pulse_count2 / encoder_slot) * ((60 * time_diff) / 5000) # 12 pulse = 1 tur
                rpm3 = (pulse_count3 / encoder_slot) * ((60 * time_diff) / 5000) # 12 pulse = 1 tur
                rpm4 = (pulse_count4 / encoder_slot) * ((60 * time_diff) / 5000) # 12 pulse = 1 tur
                print(hndshk,"we_1:",we_1,"Enc1", rpm1,"e1",error1,"|","we_2:",we_2,"Enc2", rpm2,"e2",error2,"|","we_3:",we_3,"Enc3", rpm3,"e3",error3,"|","we_4:",we_4,"Enc4", rpm4,"e4",error4,"|","Vx:",Vx,"Vy:",Vy)
                pulse_count1 = 0
                pulse_count2 = 0
                pulse_count3 = 0
                pulse_count4 = 0
                hndshk = 0
            current_time_error = utime.ticks_ms() # mevcut zaman
            time_diff_error = utime.ticks_diff(current_time_error, last_time_error)
            if time_diff_error >= 5000: # 1000ms = 1 saniye
                last_time_error = current_time_error
                error1 = 0
                error2 = 0
                error3 = 0
                error4 = 0
            error1 = abs(we_1) - rpm1 #hata hesaplanir
            error2 = abs(we_2) - rpm2 #hata hesaplanir
            error3 = abs(we_3) - rpm3 #hata hesaplanir
            error4 = abs(we_4) - rpm4 #hata hesaplanir
            pid_output1 = calculate_pid1(error1) #pid cikisini hesaplar RPM olarak cikis verir
            pid_output2 = calculate_pid2(error2) #pid cikisini hesaplar RPM olarak cikis verir
            pid_output3 = calculate_pid3(error3) #pid cikisini hesaplar RPM olarak cikis verir
            pid_output4 = calculate_pid4(error4) #pid cikisini hesaplar RPM olarak cikis verir
            motor1_rotate(pid_output1,we_1)
            motor2_rotate(pid_output2,we_2)
            motor3_rotate(pid_output3,we_3)
            motor4_rotate(pid_output4,we_4)
            
            status_led_counter = status_led_counter + 1
            sleep(0.01) #kisa bir ara :) 
        except Exception as e:
            pass
except KeyboardInterrupt:                   # trap Ctrl-C input
    terminateThread = True                  # signal second 'background' thread to terminate 
    exit()

