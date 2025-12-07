while True:
    robot.move(0, 0)
    # 無限ループ中でも、stop_program()が呼ばれると
    # ここで突然 KeyboardInterrupt が発生する
    time.sleep(0.1) 
