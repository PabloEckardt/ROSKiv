import math

angle = 0
while True:
    angle += .5
    x = math.cos((angle * math.pi) / 180) * 180
    y = math.sin((angle * math.pi) / 180) * 180

    print  (str(x) + " " + str(y) + " " + str(math.sqrt(x*x + y*y) ))
