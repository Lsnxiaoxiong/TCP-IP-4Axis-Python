import json


class Config:
    def __init__(self):
        with open("cali.json", "r") as f:
            config = json.load(f)
        try:
            self.left_top = (config["left_top"]["x"], config["left_top"]["y"])
            self.right_bottom = (config["right_bottom"]["x"], config["right_bottom"]["y"])
            self.pix_left_top = (config["pix_left_top"]["x"], config["pix_left_top"]["y"])
            self.pix_right_bottom = (config["pix_right_bottom"]["x"], config["pix_right_bottom"]["y"])
            self.press_deep = config["press_deep"]
            self.max_deep = config["max_deep"]
            self.init_pix = (config["init_pos"]["pix_x"], config["init_pos"]["pix_y"])
            self.init_pos = (config["init_pos"]["pos_x"], config["init_pos"]["pos_y"], config["init_pos"]["pos_z"])

            self.robot_on_keyboard = config["robot_on_keyboard"]
            if self.robot_on_keyboard* self.init_pos[2] > 0:
                self.robot_to_keyboard = abs(self.robot_on_keyboard-self.init_pos[2])
            else:
                self.robot_to_keyboard = abs(self.robot_on_keyboard) + abs(self.init_pos[2])
            self.cap_to_robot_end = config["cap_to_keyborad"] - self.robot_to_keyboard
        except Exception as e:
            print(e)
            raise RuntimeError("加载cali.json配置文件失败")

    @property
    def scale(self):
        y_scale = (self.pix_right_bottom[0] - self.pix_left_top[0]) / (self.right_bottom[1] - self.left_top[1])
        x_scale = (self.pix_right_bottom[1] - self.pix_left_top[1]) / (self.right_bottom[0] - self.left_top[0])
        return (y_scale + x_scale) / 2

    @property
    def scale_x(self):
        """
        机械臂x轴
        """
        return (self.pix_right_bottom[1] - self.pix_left_top[1]) / (self.right_bottom[0] - self.left_top[0])

    @property
    def scale_y(self):
        """
        机械臂y轴
        """
        return (self.pix_right_bottom[0] - self.pix_left_top[0]) / (self.right_bottom[1] - self.left_top[1])
