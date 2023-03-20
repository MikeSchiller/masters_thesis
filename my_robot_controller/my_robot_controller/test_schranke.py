# Benoetigte Module werden importiert und eingerichtet
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

# Hier wird der Eingangs-Pin deklariert, an dem der Sensor angeschlossen ist. Zusaetzlich wird auch der PullUP Widerstand am Eingang aktiviert
GPIO_PIN = 24
GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
i = 1
i = 0
start_time = 0
end_time = 0
elapsed_time = 0
turn_kurbelwelle= 0.0
turn_rad = 0.0
distance_travelled_odo = 0.0
rad_umfang = 0.12 # Umfang des Rades in [Meter]

print("Sensor-Test [druecken Sie STRG+C, um den Test zu beenden]")
print("Sensor-Test")


# Diese AusgabeFunktion wird bei Signaldetektion ausgefuehrt
def ausgabeFunktion(null):
        global start_time 
        global end_time 
        global elapsed_time 
        global turn_kurbelwelle
        global turn_rad
        global rad_umfang


        i = i+1
        turn_kurbelwelle = i/20
        turn_rad = turn_kurbelwelle/4.80808
        distance_travelled_odo = turn_rad* rad_umfang
        # vom Sensor gemessene zurückgelegte Strecke in METER
        # Wird im ROS Knoten versendet
        if i == 1000:
            start_time = time.perf_counter()
        elif i > 1000 and i< 7130:
              pass
 
# Beim Detektieren eines Signals (steigende Signalflanke) wird die Ausgabefunktion ausgeloest
GPIO.add_event_detect(GPIO_PIN, GPIO.RISING, callback=ausgabeFunktion, bouncetime=1)

# Hauptprogrammschleife
try:
    while True:
        time.sleep(0.000001)

# Aufraeumarbeiten nachdem das Programm beendet wurde
except KeyboardInterrupt:
        GPIO.cleanup()