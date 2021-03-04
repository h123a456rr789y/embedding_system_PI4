v = 343         # (331 + 0.6*20)

def measure() :
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
    time.sleep(0.00001)     # 10uS                               
    GPIO.output(TRIGGER_PIN, GPIO.LOW)
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()
    t = pulse_end - pulse_start
    d = t * v 
    d = d/2 
    return d*100
