
import time

def wait_for_seconds(sec):
    time.sleep(sec)

# sign function
def sign(value):
    if value > 0:
        return 1
    elif value < 0:
        return -1
    else:
        return 0
    


