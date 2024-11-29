import urwid

class ModeApp:
    def __init__(self, car_controller, arm_controller, custom_control):
        self.MODES = {
            'mode_vehicle': "Control Vehicle",
            'mode_arm': "Control Robotic Arm",
            'mode_nav': "Autonomous Navigation",
            'mode_auto_arm': ["Arm Mode 1", "Arm Mode 2", "Arm Mode 3"],  # 橫向子模式
        }
        self.palette = [
            ('reversed', 'standout', ''),  # 聚焦的選項
        ]
        self.car_controller = car_controller
        self.arm_controller = arm_controller
        self.custom_control = custom_control
        self.key_handler_registered = False
        self.loop = urwid.MainLoop(None, palette=self.palette)

    def main(self):
        """啟動應用程序"""
        self.main_menu()
        self.loop.run()

    def main_menu(self):
        """主選單"""
        menu_options = [
            (self.MODES['mode_vehicle'], lambda: self.switch_mode('mode_vehicle')),
            (self.MODES['mode_arm'], lambda: self.switch_mode('mode_arm')),
            (self.MODES['mode_nav'], lambda: self.switch_mode('mode_nav')),
            ("Automatic Arm Mode", lambda: self.switch_mode('mode_auto_arm')),
            ('Exit', self.exit_program),
        ]

        menu_items = [urwid.Text("Main Menu:")] + [
            urwid.AttrMap(urwid.Button(label, on_press=lambda _, func=func: func()), None, focus_map='reversed')
            for label, func in menu_options
        ]

        menu_list = urwid.ListBox(urwid.SimpleFocusListWalker(menu_items))
        self.loop.widget = menu_list

    def switch_mode(self, mode_name):
        """模式切換"""
        if mode_name == 'mode_auto_arm':
            self.horizontal_select(
                self.MODES['mode_auto_arm'],
                lambda sub_mode: self.arm_mode_handler(mode_name, sub_mode)
            )
        else:
            mode_label = self.MODES.get(mode_name, "Unknown Mode")
            text = urwid.Text(f"{mode_label}\nPress 'q' to return to the main menu.\n")
            filler = urwid.Filler(text, valign='top')

            def mode_handler(key):
                if key is None:
                    return
                if key == 'q':
                    self.main_menu()
                elif mode_name == 'mode_vehicle':
                    self.control_vehicle(key)
                elif mode_name == 'mode_arm':
                    self.control_arm(key)
                elif mode_name == 'mode_nav':
                    self.auto_navigation()

            self.loop.widget = filler
            self.loop.unhandled_input = mode_handler

    def horizontal_select(self, options, on_select):
        """橫向選擇模式"""
        index = [0]  # 用於追踪當前選擇的索引

        def render():
            """渲染橫向選單"""
            items = [
                urwid.Text(("reversed", f"[{opt}]") if i == index[0] else f" {opt} ")
                for i, opt in enumerate(options)
            ]
            return urwid.Columns(items, dividechars=1)

        def key_handler(key):
            if key == 'left':
                index[0] = (index[0] - 1) % len(options)
                self.loop.widget = urwid.Filler(render(), valign='top')
            elif key == 'right':
                index[0] = (index[0] + 1) % len(options)
                self.loop.widget = urwid.Filler(render(), valign='top')
            elif key == 'enter':
                on_select(options[index[0]])
            elif key == 'q':
                self.main_menu()

        self.loop.widget = urwid.Filler(render(), valign='top')
        self.loop.unhandled_input = key_handler

    def arm_mode_handler(self, parent_mode, sub_mode):
        """處理自動手臂子模式"""
        mode_label = f"{self.MODES[parent_mode]} - {sub_mode}"
        text = urwid.Text(f"{mode_label}\nPress 'q' to return to the selection menu.\n")
        filler = urwid.Filler(text, valign='top')

        def mode_handler(key):
            if key == 'q':
                self.switch_mode(parent_mode)  # 返回橫向選單
            else:
                self.auto_arm_mode(sub_mode, key)

        self.loop.widget = filler
        self.loop.unhandled_input = mode_handler

    def control_vehicle(self, key):
        """控制車體"""
        if key == 'q':
            self.main_menu()
        else:
            self.car_controller.manual_control(key)

    def control_arm(self, key):
        """控制機械手臂"""
        if key == 'q':
            self.main_menu()
        else:
            self.arm_controller.manual_control(key)

    def auto_navigation(self):
        """自动导航模式"""
        # Set a flag to control the auto navigation loop
        self.auto_nav_running = True

        def auto_control_loop(loop=None, user_data=None):
            if self.auto_nav_running:
                self.car_controller.auto_control(mode="auto_nav")
                # Schedule the next call to this function
                self.loop.set_alarm_in(0.1, auto_control_loop)

        # Start the auto navigation loop
        auto_control_loop()

        # Register the key handler to listen for 'q' to exit
        def key_handler(key):
            if key == 'q':  # If 'q' is pressed, stop auto navigation
                print("Exiting navigation...")
                self.auto_nav_running = False
                self.car_controller.auto_control(mode="auto_nav", key="q")
                self.main_menu()
            else:
                pass

        self.loop.unhandled_input = key_handler







    def auto_arm_mode(self, sub_mode, key):
        """自動手臂模式"""
        if key == 'q':
            self.switch_mode('mode_auto_arm')  # 返回橫向選單
        else:
            print(f"Automatic Arm Mode - {sub_mode}: Pressed {key}")

    def exit_program(self):
        """退出程序"""
        print("Exiting program.")
        raise urwid.ExitMainLoop()

def main():
    app = ModeApp()
    app.main()

if __name__ == '__main__':
    main()
