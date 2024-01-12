# import cv2
# cap=cv2.VideoCapture(2)
# while True:
#     ret, frame=cap.read()
#     cv2.imshow("Frame",frame)
#     key = cv2.waitKey(1) & 0xFF
#     if key == ord("q"):
#         cap.release()    

import cv2

def find_external_webcam_index():
    # Try indices from 0 to 10 (you can adjust the range as needed)
    for i in range(100):
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # cv2.CAP_DSHOW is used for Windows, you may need to adjust for your system
        if cap.isOpened():
            print(f"External webcam found at index {i}")
            cap.release()
            return i
    print("No external webcam found")
    return None

# Get the index of the external webcam
external_webcam_index = find_external_webcam_index()

# Use the external webcam if found
if external_webcam_index is not None:
    cap = cv2.VideoCapture(external_webcam_index, cv2.CAP_DSHOW)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Your processing or display code here

        cv2.imshow('External Webcam', frame)
        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
            break

    cap.release()
    cv2.destroyAllWindows()
