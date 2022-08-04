from io import TextIOWrapper
import sys
import glob
import serial
import time

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
    found = False
    port = ''
    s = None

    def __init__(self, samples):
        self.samples = samples
        for port in serial_ports():
            s = serial.Serial(port, 9600, timeout=1)
            print('Trying port %s.'%(port))
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
        if self.found == False:
            raise RuntimeError('Could not find Keithley Model 6485!')
        print('Using port %s.'%(self.port))

        # Set the ARM count properly.
        s.write(b'ARM:COUN 1\r')
        buf = s.read(128).decode('utf-8').rstrip()
        print(buf)

    def set_samples(self, samples):
        self.samples = samples

    def sample_data(self, pos, progbar, cidx, nidx):
        out = ''
        tot = nidx * self.samples
        frac = cidx * self.samples
        for i in range (self.samples):
            self.s.write(b'READ?\r')
            buf = self.s.read(128).decode('utf-8').rstrip()
            print(buf)
            out += str(pos) + ',' + buf + '\n'
            spbuf = buf.split(',')
            frac += 1
            progbar.emit(round(frac * 100/tot))
            # TODO: If we cannot split, the device is having errors.
            if int(float(spbuf[2])):
                print("ERROR #%d", int(float(spbuf[2])))
        return out

    def __del__(self):
        self.s.close()