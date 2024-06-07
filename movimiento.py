import Jetson.GPIO as GPIO
import time
import threading

# Configuración de pines
ENCODER_A = 18  
ENCODER_B = 23  
BUTTON_MOD = 24  

POT_PIN = 0    

M1Pin = 16
M2Pin = 21

# Variable global de posición compartida con la interrupción
theta = 0

# Variable global de pulsos compartida con la interrupción
pulsos = 0
timeold = 0
resolution = 341.22

# Variable Global Velocidad
vel = 0

# Variable Global Posicion
ang = 0

# Variable Global MODO
modo = False

# Configuración de GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A, GPIO.IN)
GPIO.setup(ENCODER_B, GPIO.IN)
GPIO.setup(BUTTON_MOD, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(M1Pin, GPIO.OUT)
GPIO.setup(M2Pin, GPIO.OUT)

# Configuración del PWM
pwm = GPIO.PWM(M1Pin, 30.64)
pwm.start(0)

# Función para leer el valor del potenciómetro (simulación)
def read_pot():
    # Simulación de la lectura del potenciómetro
    return 512  # Valor medio para la simulación

# Función para configurar el motor
def setMotor(vel, dir):
    pwm.ChangeDutyCycle(vel)
    if dir:
        GPIO.output(M1Pin, GPIO.LOW)
    else:
        GPIO.output(M1Pin, GPIO.HIGH)

# Función anti-rebote
def debounce(input_pin):
    state = False
    if not GPIO.input(input_pin):
        time.sleep(0.2)
        while not GPIO.input(input_pin):
            pass
        time.sleep(0.2)
        state = True
    return state

# Función para la lectura del encoder
def leerEncoder(channel):
    global pulsos, theta
    if modo:
        pulsos += 1  # Incrementa una revolución
    else:
        b = GPIO.input(ENCODER_B)
        if b > 0:
            theta += 1
        else:
            theta -= 1

# Configurar interrupción
GPIO.add_event_detect(ENCODER_A, GPIO.RISING, callback=leerEncoder)

def main():
    global timeold, vel, ang, modo, theta, pulsos
    while True:
        posicion = 0.0
        rpm = 0.0
        value = read_pot()
        dir = True

        # Cambia de Modo Velocidad o Posición
        if debounce(BUTTON_MOD):
            modo = not modo
            theta = 0

        if modo:
            # Transforma el valor del Pot a velocidad
            vel = int(value * 255 / 1023)

            # Activa el motor dirección Forward con la velocidad
            setMotor(vel, False)

            # Espera un segundo para el cálculo de las RPM
            if time.time() * 1000 - timeold >= 1000:
                with threading.Lock():
                    rpm = float((60.0 * 1000.0 / resolution) / (time.time() * 1000 - timeold) * pulsos)
                    timeold = time.time() * 1000
                    pulsos = 0
                print("RPM: ", rpm)
                print("PWM: ", vel)
        else:
            # Transforma el valor del Pot a ángulo
            ang = int(value * 360 / 1023)

            with threading.Lock():
                posicion = float(theta * 360.0 / resolution)

            # Posiciona el ángulo con tolerancia +- 2
            if ang > posicion + 2:
                vel = 200
                dir = True
            elif ang < posicion - 2:
                vel = 200
                dir = False
            else:
                vel = 0
            setMotor(vel, dir)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()

