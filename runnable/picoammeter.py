from io import TextIOWrapper
import sys
import glob
import serial
from time import sleep

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

class Picoammeter:
    def __init__(self, samples: int):
        if samples < 2:
            samples = 2
        if samples > 20:
            samples = 20
        self.samples = samples
        self.s = None
        self.found = False
        self.port = -1
        for port in serial_ports():
            s = serial.Serial(port, 9600, timeout=1)
            print('Trying port %s.'%(port))
            s.write(b'*RST\r')
            sleep(0.5)
            s.write(b'*IDN?\r')
            buf = s.read(128).decode('utf-8').rstrip()
            print(buf)

            if 'KEITHLEY INSTRUMENTS INC.,MODEL 6485' in buf:
                print("Keithley Model 6485 found.")
                self.found = True
                self.port = port
                self.s = s
            else:
                print("Keithley Model 6485 not found.")
                s.close()

        if self.found == False:
            raise RuntimeError('Could not find Keithley Model 6485!')
        print('Using port %s.'%(self.port))

        self.s.write(b'SYST:ZCH ON\r')
        sleep(0.1)
        # buf = s.read(128).decode('utf-8').rstrip()
        # print('SYST:ZCH ON: %s'%(buf))

        self.s.write(b'RANG 2e-9\r')
        sleep(0.1)
        # buf = s.read(128).decode('utf-8').rstrip()

        self.s.write(b'INIT\r')
        sleep(0.1)

        self.s.write(b'SYST:ZCOR:ACQ\r') # acquire zero current
        sleep(0.1)

        self.s.write(b'SYST:ZCOR ON\r') # perform zero correction
        sleep(0.1)

        self.s.write(b'RANG:AUTO ON\r') # enable auto range
        sleep(0.1)

        self.s.write(b'SYST:ZCH OFF\r') # disable zero check
        sleep(0.1)

        self.s.write(b'SYST:ZCOR OFF\r') # disable zero correction
        sleep(0.1)

        self.s.write(b'AVER ON\r')
        self.s.write(b'AVER:TCON REP\r')
        self.s.write(b'AVER:COUN %d\r'%(self.samples)) # enable averaging
        print('Init complete')


    def set_samples(self, samples: int):
        if samples < 2:
            samples = 2
        if samples > 20:
            samples = 20
        self.samples = samples
        self.s.write(b'AVER:COUN %d\r'%(self.samples)) # enable averaging

    def sample_data(self):
        out = ''
        self.s.write(b'READ?\r')
        retry = 10
        while retry:
            buf = self.s.read(128).decode('utf-8').rstrip()
            if len(buf):
                break
            retry -= 1
        print(buf)
        spbuf = buf.split(',')
        try:
            if int(float(spbuf[2])) != 2:
                print("ERROR #%d"%(int(float(spbuf[2]))))
        except Exception:
            print('Error: %s invalid output'%(buf))
        return out

    def __del__(self):
        if self.s is not None:
            self.s.close()

# test code

if __name__ == '__main__':
    import sys
    import signal

    done = False

    def signal_handler(sig, frame):
        global done
        print('You pressed Ctrl+C!')
        done = True

    signal.signal(signal.SIGINT, signal_handler)

    pa = Picoammeter(3)
    while not done:
        print(pa.sample_data())

    sys.exit(0)

    