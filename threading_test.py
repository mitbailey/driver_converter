# %%
import sys
from threading import Thread, Lock, Condition
from time import sleep

class Test:
    def __init__(self, input: str) -> None:
        self.message = input
        self.lock = Lock()
        self.cond = Condition(self.lock)

    def print_msg_internal(self):
        print(self.message)
        print('Wait 10 seconds...')
        sleep(10)
        with self.cond:
            self.cond.notify()
            print('Notified')

    def get_msg(self):
        thr = Thread(target = self.print_msg_internal)
        thr.start()
        print('Waiting on thread...')
        with self.cond:
            self.cond.wait()
            print('Condition triggered')
        thr.join()
        print('Thread joined')

if __name__ == '__main__':
    a = Test('Hello world!')
    a.get_msg()

    sys.exit(0)