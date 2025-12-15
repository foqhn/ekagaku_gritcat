res_improc = None


while True:
  robot.capture_image()
  res_improc = robot.detect_color_centroid('HSV', [10, 100, 100], [40, 255, 255])
  robot.draw_marker(res_improc['x'], res_improc['y'], color=(0, 255, 0), size=30)
  robot.show_image()
