import urwid
import threading

class ModeApp:
    def __init__(self, car_controller, arm_controller, custom_control, crane_controller):
        self.MODES = {
            'mode_vehicle': "Control Vehicle",
            'mode_arm': "Control Robotic Arm",
            'mode_crane': "Control Crane",
            'mode_auto_nav': ["manual_auto_nav", "target_auto_nav"],
            'mode_auto_arm': ["Arm Mode 1", "Arm Mode 2", "Arm Mode 3"],  # 橫向子模式
        }
        self.palette = [
            ('reversed', 'standout', ''),  # 聚焦的選項
        ]
        self.car_controller = car_controller
        self.arm_controller = arm_controller
        self.custom_control = custom_control
        self.crane_controller = crane_controller
        self.loop = urwid.MainLoop(None, palette=self.palette)

    def main(self):
        self.main_menu()
        self.loop.run()

    def main_menu(self):
        """主選單"""
        menu_options = [
            (self.MODES['mode_vehicle'], lambda: self.switch_mode('mode_vehicle')),
            (self.MODES['mode_arm'], lambda: self.switch_mode('mode_arm')),
            (self.MODES['mode_crane'], lambda: self.switch_mode('mode_crane')),
            ("Auto Navigation", lambda: self.switch_mode('mode_auto_nav')),
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
                lambda sub_mode: self.arm_mode_handler(mode_name, sub_mode, is_auto_mode=True)
            )
        elif mode_name == 'mode_auto_nav':
            self.horizontal_select(
                self.MODES['mode_auto_nav'],
                lambda sub_mode: self.nav_mode_handler(mode_name, sub_mode)
            )
        elif mode_name == 'mode_arm':
            self.horizontal_select(
                ["0", "1", "2", "3", "4"],
                lambda sub_mode: self.arm_mode_handler(mode_name, sub_mode, is_auto_mode=False)
            )
        elif mode_name == 'mode_crane':
            self.horizontal_select(
                ["0", "1", "2", "3", "4", "5", "6","99"],
                lambda sub_mode: self.crane_mode_handler(mode_name, sub_mode, is_auto_mode=False)
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
                # elif mode_name == 'mode_arm':
                #     self.control_arm(key)
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

    def arm_mode_handler(self, parent_mode, sub_mode, is_auto_mode=False):
        """Handles both manual and automatic arm sub-modes."""
        self.sub_mode = sub_mode
        mode_type = "Automatic" if is_auto_mode else "Manual"
        mode_label = f"{self.MODES[parent_mode]} - {sub_mode} ({mode_type})"
        text = urwid.Text(f"{mode_label}\nPress 'q' to return to the selection menu.\n")
        filler = urwid.Filler(text, valign='top')

        def mode_handler(key):
            if key == 'q':
                self.clean_terminal()
                self.switch_mode(parent_mode)  # Return to horizontal menu
            else:
                if is_auto_mode:
                    self.auto_arm_mode(self.sub_mode, key)
                else:
                    self.sub_mode = int(self.sub_mode)
                    self.manual_arm_mode(self.sub_mode, key)

        self.loop.widget = filler
        self.loop.unhandled_input = mode_handler

    def crane_mode_handler(self, parent_mode, sub_mode, is_auto_mode=False):
        """Handles both manual and automatic crane sub-modes."""
        self.sub_mode = sub_mode
        mode_type = "Automatic" if is_auto_mode else "Manual"
        mode_label = f"{self.MODES[parent_mode]} - {sub_mode} ({mode_type})"
        text = urwid.Text(f"{mode_label}\nPress 'q' to return to the selection menu.\n")
        filler = urwid.Filler(text, valign='top')

        def mode_handler(key):
            if key == 'q':
                self.clean_terminal()
                self.switch_mode(parent_mode)  # Return to horizontal menu
            else:
                self.sub_mode = int(self.sub_mode)
                self.manual_crane_mode(self.sub_mode, key)

        self.loop.widget = filler
        self.loop.unhandled_input = mode_handler
    def nav_mode_handler(self, parent_mode, sub_mode):
        """處理自動導航子模式"""
        mode_label = f"{self.MODES[parent_mode]} - {sub_mode}"
        text = urwid.Text(f"{mode_label}\nPress 'q' to return to the selection menu.\n")
        filler = urwid.Filler(text, valign='top')

        def mode_handler(key):
            if key == 'q':
                self.switch_mode(parent_mode)
            else:
                # print(f"Automatic navigation Mode - {sub_mode}: Pressed {key}")
                # self.car_controller.auto_control(mode=sub_mode, key=key)
                # self.auto_nav_mode(sub_mode, key)
                self.auto_navigation(parent_mode=parent_mode)

        self.loop.widget = filler
        self.loop.unhandled_input = mode_handler
        
        self.auto_navigation(parent_mode=parent_mode)

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

    def auto_navigation(self, parent_mode):
        """auto navigation mode"""
        # Set a flag to control the auto navigation loop
        self.auto_nav_running = True

        def auto_control_loop(loop=None, user_data=None):
            if self.auto_nav_running:
                self.car_controller.auto_control(mode="manual_auto_nav")
                # Schedule the next call to this function
                self.loop.set_alarm_in(0.1, auto_control_loop)
        auto_control_loop()
        def key_handler(key):
            if key == 'q':  # If 'q' is pressed, stop auto navigation
                print("Exiting navigation...")
                self.clean_terminal()
                self.auto_nav_running = False
                self.car_controller.auto_control(mode="manual_auto_nav", key="q")
                # self.main_menu()
                self.switch_mode(parent_mode)
            else:
                self.car_controller.auto_control(mode="manual_auto_nav", key=key)
        self.loop.unhandled_input = key_handler

    def clean_terminal(self):
        print("\033[2J\033[H", end="")  # 清除螢幕並將光標移動到頂部

    def auto_arm_mode(self, sub_mode, key):
        """自動手臂模式"""
        if key == 'q':
            self.clean_terminal()
            self.switch_mode('mode_auto_arm')  # 返回橫向選單
        else:
            print(f"Automatic Arm Mode - {sub_mode}: Pressed {key}")

    def manual_arm_mode(self, sub_mode, key):
        """手動手臂模式"""
        if key == 'q':
            self.clean_terminal()
            self.switch_mode('mode_arm')  # 返回橫向選單
        else:
            self.arm_controller.manual_control(sub_mode, key)
    
    def manual_crane_mode(self, sub_mode, key):
        """手動天車模式"""
        if key == 'q':
            self.clean_terminal()
            self.switch_mode('mode_crane')  # 返回橫向選單
        else:
            self.crane_controller.manual_control(sub_mode, key)
        
    def auto_nav_mode(self, sub_mode, key):
        """自動手臂模式"""
        if key == 'q':
            self.switch_mode('mode_auto_nav')
        else:
            print(f"Automatic navigation Mode - {sub_mode}: Pressed {key}")

    def exit_program(self):
        """退出程序"""
        print("Exiting program.")
        raise urwid.ExitMainLoop()

def main():
    app = ModeApp()
    app.main()

if __name__ == '__main__':
    main()
