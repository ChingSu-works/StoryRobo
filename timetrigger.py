from threading import Timer

def print_hello():
     print("hello world")

class RepeatingTimer(Timer): 
    def run(self):
        self.finished.wait(self.interval)
        while not self.finished.is_set():
            self.function(*self.args, **self.kwargs)
            self.finished.wait(self.interval)

t = RepeatingTimer(0.1, print_hello)
t.start()