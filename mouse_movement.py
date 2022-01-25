import mouse
import serial

DIVISOR = .7
MOVE_DURATION = .035

def get_signed_coord(c):
  if c < 128:
    return -c
  else:
    return 256 - c

stm = serial.Serial('/dev/ttyACM0', timeout=.01)

# Get rid of accumulated coords
stm.readline()
stm.read_until(b')')

while True:
  data = stm.read_until(b')')
  
  if data:
    # print(data)
    data = data.decode()
    
    if data[-1] != ')' or data[0] != '(':
      continue

    # print(data)
    data = data[1:-1]
    coords = data.split(',')
    try:
      x = int(coords[0])
      y = int(coords[1])
      
      x = get_signed_coord(x)
      y = get_signed_coord(y)

      mouse.move(x / DIVISOR, -y / DIVISOR, absolute=False, duration=MOVE_DURATION)

      # print(x, y)
    except:
      pass
