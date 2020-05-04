from threading import Timer
from functools import partial

class Interval(object):

    def __init__(self, interval, function, args=[], kwargs={}):
        """
        Runs the function at a specified interval with given arguments.
        """
        self.interval = interval
        self.function = partial(function, *args, **kwargs)
        self.running  = False 
        self._timer   = None 

    def __call__(self):
        """
        Handler function for calling the partial and continuting. 
        """
        self.running = False  # mark not running
        self.start()          # reset the timer for the next go 
        self.function()       # call the partial function 

    def start(self):
        """
        Starts the interval and lets it run. 
        """
        if self.running:
            # Don't start if we're running! 
            return 
            
        # Create the timer object, start and set state. 
        self._timer = Timer(self.interval, self)
        self._timer.start() 
        self.running = True

    def stop(self):
        """
        Cancel the interval (no more function calls).
        """
        if self._timer:
            self._timer.cancel() 
        self.running = False 
        self._timer  = None


if __name__ == "__main__":
    import time 
    import random
    
    def clock(start):
        """
        Prints out the elapsed time when called from start.
        """
        print("elapsed: {:0.3f} seconds".format(time.time() - start))

    # Create an interval. 
    interval = Interval(random.randint(1,3), clock, args=[time.time(),])
    print("Starting Interval, press CTRL+C to stop.")
    interval.start() 

    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("Shutting down interval ...")
            interval.stop()
            break
