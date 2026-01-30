from PythonExample import connect_robot

if __name__ == "__main__":
    dashboard, move, feed = connect_robot()

    # J1 = 0.1
    # J2 = 0.1
    # J3 = 0.1
    # J4 = 0.1
    # User = 1
    # Tool = 1
    # res = dashboard.InverseSolution(J1, J2, J3, J4, User, Tool)
    # print(res)
    move.MovL(20, 280, -60, 200)