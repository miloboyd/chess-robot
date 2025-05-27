import cv2



def find_center_of_mass(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    ret, thresh = cv2.threshold(gray, 127, 255, 0)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



    for cnt in contours:

        M = cv2.moments(cnt)

        cx = int(M['m10'] / M['m00'])

        cy = int(M['m01'] / M['m00'])

        

        # Draw a circle at the center of mass

        cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1) 



    return image



# Load an image

img = cv2.imread("pawns/pawn3.png")



# Find and draw center of mass

result = find_center_of_mass(img)

cv2.imshow('Result', result)

cv2.waitKey(0)

cv2.destroyAllWindows()
