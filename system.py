"""system.py

   This is the main script that coordinates both the controller and
   the detector.  

"""
# Import the system parts
import threading
import traceback
from controller import controller
from detector import detector

#
#  Shared Data
#
class Shared:

    def __init__(self):
        # Thread Lock.  Always acquire() this lock before accessing
        # anything else (either reading or writing) in this object.
        # And don't forget to release() the lock when done!
        self.lock = threading.Lock()
        self.newdata = False
        # Flag - stop the detection.  If this is set to True, the
        # detection should break out of the loop and stop.
        self.stop = False

        # Motor data
        self.motorpan  = 0.0
        self.motortilt = 0.0

        # Object Data - PLEASE UPDATE TO ADD THE DATA YOU NEED!
        self.objectpan = 0
        self.objecttilt = 0
        self.detectedobjects = []


#
#  Main Code
#
def main():
    # Prepare a single instance of the shared data object.
    shared = Shared()

    # Create a second thread.
    thread = threading.Thread(target=detector, args=(shared,))

    # Start the second thread with the detector.
    print("Starting second thread")
    thread.start()      # Equivalent to detector(shared) in new thread

    # Use the primary thread for the controller, handling exceptions
    # to gracefully to shut down.
    try:
        controller(shared)
    except BaseException as ex:
        # Report the exception
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Stop/rejoin the second thread.
    print("Stopping second thread...")
    if shared.lock.acquire():
        shared.stop = True
        shared.lock.release()
    thread.join()       # Wait for thread to end and re-combine.

if __name__ == "__main__":
    main()
