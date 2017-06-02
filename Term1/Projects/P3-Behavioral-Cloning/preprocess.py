import cv2

### Preprocess input images according to NVIDIA doc
def preprocess_input(image):
    processed = image[60:140, :] # Removes unuseful information from the image (trees, sky, burds..)
    processed = cv2.resize(image, (200,66), interpolation = cv2.INTER_AREA)
    processed = cv2.cvtColor(processed, cv2.COLOR_RGB2YUV)
    return processed