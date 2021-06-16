import time
import serial
import sys,tty,termios
class _Getch:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def get():
    inkey = _Getch()
    while(1):
        k=inkey()
        if k!='':break
    if k=='\x1b':
        k2 = inkey()
        k3 = inkey()
        if k3=='A':
            print ("up")
            s.write("/CT/run 100 \n".encode())
        if k3=='B':
            print ("down")
            s.write("/CT/run -100 \n".encode())
        if k3=='C':
            print ("right")
            s.write("/turn/run 100 -0.1 \n".encode())
        if k3=='D':
            print ("left")
            s.write("/turn/run 100 0.1 \n".encode())
        time.sleep(1)
        s.write("/stop/run \n".encode())
    elif k=='1':
        print ("region: 1")
        k4 = input()
        if k4 == 'N':
            print("director: N")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 1 0", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'NW':
            print("director: NW")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 1 7", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'W':
            print("director: W")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 1 6", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'SW':
            print("director: SW")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 1 5", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'S':
            print("director: S")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 1 4", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'NE':
            print("director: NE")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 1 1", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'E':
            print("director: E")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 1 2", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'SE':
            print("director: SE")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 1 3", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
    elif k=='2':
        print ("region: 2")
        k4 = input()
        if k4 == 'N':
            print("director: N")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 2 0", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'NW':
            print("director: NW")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 2 7", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'W':
            print("director: W")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 2 6", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'SW':
            print("director: SW")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 2 5", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'S':
            print("director: S")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 2 4", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'NE':
            print("director: NE")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 2 1", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'E':
            print("director: E")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 2 2", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
        elif k4 == 'SE':
            print("director: SE")
            k5 = input()
            d1 = float(k5)
            k6 = input()
            d2 = float(k6)
            print_args = ("/car/run 2 3", d1, d2)
            s.write(("%s %f %f \n" % print_args).encode())
    elif k=='q':
        print ("quit")
        return 0
    else:
        print ("not an arrow key!")
    return 1

if len(sys.argv) < 1:
    print ("No port input")
s = serial.Serial(sys.argv[1])
while get():
    i = 0