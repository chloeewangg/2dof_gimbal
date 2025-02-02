'''goals7detector.py

   This encapsulates the object detector code into a single function.
'''

#
#   Define your entire code as a function.  The argument will be the
#   shared data that is accessed by both pieces.
#
import cv2

def detector(shared):
    # Place all your code here.  Note you will have to indent EVERYTHING.
    camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise Exception(
        "Could not open video device: Maybe change the cam number?")

# Change the frame size and rate.  Note only combinations of
# widthxheight and rate are allowed.  In particular, 1920x1080 only
# reads at 5 FPS.  To get 30FPS we downsize to 640x480.
    camera.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS,           30)

# scale factors for object tracking
    scale_pan = -0.00147078307
    #scale_tilt = -0.0015
    scale_tilt = -0.00060742317

# Change the camera settings.
    exposure = 163
    wb = 4614
    focus = 0

# camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)       # Auto mode
    camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)       # Manual mode
    camera.set(cv2.CAP_PROP_EXPOSURE, exposure)     # 3 - 2047, default 250

# camera.set(cv2.CAP_PROP_AUTO_WB, 1.0)           # Enable auto white balance
    camera.set(cv2.CAP_PROP_AUTO_WB, 0.0)           # Disable auto white balance
    camera.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)     # 2000 - 6500, default 4000

# camera.set(cv2.CAP_PROP_AUTOFOCUS, 1)           # Enable autofocus
    camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)           # Disable autofocus
    camera.set(cv2.CAP_PROP_FOCUS, focus)           # 0 - 250, step 5, default 0

    camera.set(cv2.CAP_PROP_BRIGHTNESS, 128)        # 0 - 255, default 128
    camera.set(cv2.CAP_PROP_CONTRAST,   32)        # 0 - 255, default 128
    camera.set(cv2.CAP_PROP_SATURATION, 32)        # 0 - 255, default 128


# Keep scanning, until 'q' hit IN IMAGE WINDOW.
    count = 0
    while True:
        # Grab an image from the camera.  Often called a frame (part of sequence).
        ret, frame = camera.read()
        count += 1

        # Grab and report the image shape.
        (H, W, D) = frame.shape
        # print(f"Frame #{count:3} is {W}x{H} pixels x{D} color channels.")

        # Convert the BGR image to RGB or HSV.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)      # For other objects
        # hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)      # For red objects

        hmin = 102
        hmax = 111
        smin = 90
        smax = 175
        vmin = 95
        vmax = 255

        iter = 2
        binary = cv2.inRange(hsv, (hmin, smin, vmin), (hmax, smax, vmax))
        binary = cv2.dilate(binary, None, iterations = iter)
        binary = cv2.erode(binary, None, iterations = 4 * iter)
        binary = cv2.dilate(binary, None, iterations = 3 * iter)
    

        # Print color of center pixel
        g = frame[H//2, W//2, 0]
        b = frame[H//2, W//2, 1]
        r = frame[H//2, W//2, 2]
        center_p_gbr = ((g, b, r))

        h = hsv[H//2, W//2, 0]
        s = hsv[H//2, W//2, 1]
        v = hsv[H//2, W//2, 2]
        center_p_hsv = ((h, s, v))

        framexc = W//2
        frameyc = H//2

        cv2.line(frame, (0, H//2), (W, H//2), (0, 255, 255), 1)
        cv2.line(frame, (W//2, 0), (W//2, H), (0, 255, 255), 1)
        
        (contours, hierarchy) = cv2.findContours(
        binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        detectedobjects = []
        cutoff = 4000

        for contour in contours:
            if cv2.contourArea(contour) > cutoff:
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)
                ((xr, yr), radius) = cv2.minEnclosingCircle(contour)

                cv2.circle(frame, (int(xr), int(yr)), 4, (0, 255, 0), -1)
                #print((xr, yr))

                if shared.lock.acquire():
                    camerapan = shared.motorpan
                    cameratilt = shared.motortilt

                    objectpan = camerapan + scale_pan*(int(xr)-framexc)
                    objecttilt = cameratilt + scale_tilt*(int(yr)-frameyc)

                    detectedobjects.append((objectpan, objecttilt))
                    
                    shared.detectedobjects = detectedobjects.copy()
                    shared.newdata = True
                    shared.objecttilt = objecttilt
                    shared.objectpan = objectpan

                    #print("objectpan: ",objectpan)
                    #print("objecttilt: ",objecttilt)
                    shared.lock.release()

    # Show the processed image with the given title. 
        cv2.imshow('Processed Image', frame)
        cv2.imshow('Binary Image', binary)

    # Check for a key press IN THE IMAGE WINDOW: waitKey(0) blocks
    # indefinitely, waitkey(1) blocks for at most 1ms.  If 'q' break.
    # This also flushes the windows and causes it to actually appear.
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break
        if shared.lock.acquire():
            stop = shared.stop
            shared.lock.release()
            if stop:
                break

# Close everything up.
    camera.release()
    cv2.destroyAllWindows()


#
#   Test
#
#   If you run "python3 goals7detector.py" this will be executed.
#
if __name__ == "__main__":
    detector(None)
