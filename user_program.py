res_improc = None


while True:
  robot.capture_image()
  res_improc = robot.detect_color_centroid('HSV', [0, 100, 100], [10, 255, 255])
  robot.print_display(res_improc)
  robot.draw_marker(res_improc['x'], res_improc['y'], color=(0, 255, 0), size=20)
  robot.show_image()
