# Benoetigte Module werden importiert und eingerichtet
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Hier wird der Eingangs-Pin deklariert, an dem der Sensor angeschlossen ist. Zusaetzlich wird auch der PullUP Widerstand am Eingang aktiviert
GPIO_PIN = 24
GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
i = 1
start_time = 0
end_time = 0
elapsed_time = 0

print("Sensor-Test [druecken Sie STRG+C, um den Test zu beenden]")

#test für git
# Diese AusgabeFunktion wird bei Signaldetektion ausgefuehrt
def ausgabeFunktion(null):
        global i
        global start_time 
        global end_time 
        global elapsed_time 
        i = i+1
        if i == 1000:
            start_time = time.perf_counter()
        elif i > 1000 and i< 7130:
            print(i)
            #i/6 = Umdrehungen Kurbelwelle
        elif i ==  7130:
            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            print("Elapsed time: ", elapsed_time)
 
# Beim Detektieren eines Signals (steigende Signalflanke) wird die Ausgabefunktion ausgeloest
GPIO.add_event_detect(GPIO_PIN, GPIO.RISING, callback=ausgabeFunktion, bouncetime=1)

# Hauptprogrammschleife
try:
    while True:
        time.sleep(0.000001)

# Aufraeumarbeiten nachdem das Programm beendet wurde
except KeyboardInterrupt:
        GPIO.cleanup()