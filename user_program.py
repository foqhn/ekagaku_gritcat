img = None
mask = None
cx = None
cy = None


while True:
  img = robot.camera()
  mask = vision.color_mask(img, 'red')
  cx = vision.largest_region(mask)['Centroid'][0]
  cy = vision.largest_region(mask)['Centroid'][1]
  img = vision.insertMarker(img, (cx, cy), 'green')
  vision.imshow(img)
  robot.sleep(0.05)
